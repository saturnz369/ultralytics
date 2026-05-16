#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"
INFER_CONFIG="${INFER_CONFIG:-${SCRIPT_DIR}/config/config_infer_primary_yolo26.txt}"
TRACKER_CONFIG="${TRACKER_CONFIG:-${SCRIPT_DIR}/config/tracker_config.txt}"

yaml_quote() {
    local value="${1:-}"
    value="${value//\\/\\\\}"
    value="${value//\"/\\\"}"
    printf '"%s"' "${value}"
}

infer_cfg_value() {
    local key="$1"
    local path="$2"
    [[ -f "${path}" ]] || return 0
    grep -m1 "^${key}=" "${path}" 2>/dev/null | cut -d'=' -f2-
}

PYTHON_BIN="${PYTHON_BIN:-/home/saturnzzz/DeepStream-Yolo/.venv-yolo26-sys/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    PYTHON_BIN="python3"
fi

SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/ttyUSB0}"
if [[ "${THORNG_DIALOUT_REEXEC:-0}" != "1" ]] && [[ -n "${SERIAL_DEVICE}" ]] && [[ -e "${SERIAL_DEVICE}" ]]; then
    if [[ ! -r "${SERIAL_DEVICE}" || ! -w "${SERIAL_DEVICE}" ]]; then
        if command -v sg >/dev/null 2>&1 && getent group dialout | grep -qw "$(id -un)"; then
            printf -v SELF_CMD '%q ' "$0" "$@"
            echo "Re-launching under dialout group for ${SERIAL_DEVICE} access..."
            exec sg dialout -c "THORNG_DIALOUT_REEXEC=1 bash ${SELF_CMD}"
        fi
    fi
fi

SENSOR_ID="${SENSOR_ID:-0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-2028}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-1112}"
CAMERA_FPS_N="${CAMERA_FPS_N:-60}"
CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
METADATA_MAX_AGE_MS="${METADATA_MAX_AGE_MS:-150}"
SHOW="${SHOW:-0}"
PRINT_FRAME_LOGS="${PRINT_FRAME_LOGS:-0}"
TARGET_CLASS_ID="${TARGET_CLASS_ID:-0}"
SELECTION="${SELECTION:-center}"
TARGET_ID="${TARGET_ID:-}"
LOST_BUFFER="${LOST_BUFFER:-15}"
PAN_GAIN="${PAN_GAIN:-0.91}"
TILT_GAIN="${TILT_GAIN:-0.83}"
DEADZONE="${DEADZONE:-0.048}"
SMOOTH_ALPHA="${SMOOTH_ALPHA:-0.39}"
FAST_SMOOTH_ALPHA="${FAST_SMOOTH_ALPHA:-0.81}"
FAST_ERROR_ZONE="${FAST_ERROR_ZONE:-0.15}"
COMMAND_BOOST_ZONE="${COMMAND_BOOST_ZONE:-0.095}"
MIN_ACTIVE_COMMAND="${MIN_ACTIVE_COMMAND:-0.20}"
RESPONSE_GAMMA="${RESPONSE_GAMMA:-0.63}"
PAN_FEEDFORWARD_GAIN="${PAN_FEEDFORWARD_GAIN:-0.18}"
TILT_FEEDFORWARD_GAIN="${TILT_FEEDFORWARD_GAIN:-0.12}"
FEEDFORWARD_ALPHA="${FEEDFORWARD_ALPHA:-0.40}"
FEEDFORWARD_LIMIT="${FEEDFORWARD_LIMIT:-0.14}"
FEEDFORWARD_ACTIVATION_ZONE="${FEEDFORWARD_ACTIVATION_ZONE:-0.07}"
MAX_COMMAND="${MAX_COMMAND:-1.0}"
INVERT_PAN="${INVERT_PAN:-0}"
INVERT_TILT="${INVERT_TILT:-0}"
RTSP_ENABLE="${RTSP_ENABLE:-1}"
RTSP_PORT="${RTSP_PORT:-8554}"
RTSP_MOUNT="${RTSP_MOUNT:-/stream}"
UDP_PORT="${UDP_PORT:-5400}"
BITRATE="${BITRATE:-8000000}"
RAW_RECORD_ENABLE="${RAW_RECORD_ENABLE:-0}"
RAW_RECORD_FILE="${RAW_RECORD_FILE:-}"
RAW_RECORD_BITRATE="${RAW_RECORD_BITRATE:-40000000}"
VIDEO_QUEUE_BUFFERS="${VIDEO_QUEUE_BUFFERS:-2}"
RTSP_QUEUE_BUFFERS="${RTSP_QUEUE_BUFFERS:-4}"
RAW_RECORD_QUEUE_BUFFERS="${RAW_RECORD_QUEUE_BUFFERS:-8}"
ALLOW_MAX_FRAMES="${ALLOW_MAX_FRAMES:-0}"
if [[ "${ALLOW_MAX_FRAMES}" == "1" ]]; then
    MAX_FRAMES="${MAX_FRAMES:-0}"
else
    if [[ -n "${MAX_FRAMES+x}" && "${MAX_FRAMES}" != "0" ]]; then
        echo "Ignoring inherited MAX_FRAMES=${MAX_FRAMES}; forcing unlimited run. Set ALLOW_MAX_FRAMES=1 to use bounded runs."
    fi
    MAX_FRAMES=0
fi
METADATA_STATE_FILE="${METADATA_STATE_FILE:-}"
STATE_FILE_FLUSH="${STATE_FILE_FLUSH:-0}"
METADATA_IPC_FILE="${METADATA_IPC_FILE:-}"
DEEPSTREAM_LOG_FILE="${DEEPSTREAM_LOG_FILE:-}"
SHOW_DEEPSTREAM_LOG="${SHOW_DEEPSTREAM_LOG:-0}"

SERIAL_BAUD="${SERIAL_BAUD:-921600}"
MAV_SOURCE_SYSTEM="${MAV_SOURCE_SYSTEM:-42}"
MAV_SOURCE_COMPONENT="${MAV_SOURCE_COMPONENT:-191}"
MAV_TARGET_SYSTEM="${MAV_TARGET_SYSTEM:-1}"
MAV_TARGET_COMPONENT="${MAV_TARGET_COMPONENT:-154}"
GIMBAL_DEVICE_ID="${GIMBAL_DEVICE_ID:-154}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-5.0}"
ACK_TIMEOUT="${ACK_TIMEOUT:-2.0}"
SEND_RATE_HZ="${SEND_RATE_HZ:-20.0}"
POLL_INTERVAL="${POLL_INTERVAL:-0.005}"
MAX_YAW_RATE_DPS="${MAX_YAW_RATE_DPS:-95.0}"
MAX_PITCH_RATE_DPS="${MAX_PITCH_RATE_DPS:-60.0}"
LIVE_CONTROL_MODE="${LIVE_CONTROL_MODE:-angle-target}"
INITIAL_YAW_DEG="${INITIAL_YAW_DEG:-0.0}"
INITIAL_PITCH_DEG="${INITIAL_PITCH_DEG:-0.0}"
MAX_YAW_ANGLE_DEG="${MAX_YAW_ANGLE_DEG:-90.0}"
MIN_PITCH_ANGLE_DEG="${MIN_PITCH_ANGLE_DEG:--45.0}"
MAX_PITCH_ANGLE_DEG="${MAX_PITCH_ANGLE_DEG:-25.0}"
YAW_LOCK="${YAW_LOCK:-0}"
PITCH_LOCK="${PITCH_LOCK:-0}"
MAV_INVERT_PAN="${MAV_INVERT_PAN:-0}"
MAV_INVERT_TILT="${MAV_INVERT_TILT:-0}"
SKIP_CONFIGURE="${SKIP_CONFIGURE:-0}"
SECONDARY_CONTROL_SYSTEM="${SECONDARY_CONTROL_SYSTEM:--1}"
SECONDARY_CONTROL_COMPONENT="${SECONDARY_CONTROL_COMPONENT:--1}"
DRY_RUN_MAVLINK="${DRY_RUN_MAVLINK:-0}"
CONTROL_API="${CONTROL_API:-command}"
PRINT_STATE="${PRINT_STATE:-0}"
BRIDGE_STATE_FILE="${BRIDGE_STATE_FILE:-}"
PRINT_HEALTH="${PRINT_HEALTH:-0}"
HEALTH_PRINT_INTERVAL_SEC="${HEALTH_PRINT_INTERVAL_SEC:-1.0}"
HEALTH_STATE_FILE="${HEALTH_STATE_FILE:-}"
HEALTH_TEMP_ZONE="${HEALTH_TEMP_ZONE:-tj-thermal}"
RUN_ARTIFACTS_ENABLE="${RUN_ARTIFACTS_ENABLE:-1}"
RUNS_ROOT="${RUNS_ROOT:-${SCRIPT_DIR}/runs}"
RUN_TAG="${RUN_TAG:-$(date +%Y%m%d-%H%M%S)}"
RUN_DIR=""
CONFIG_USED_FILE=""
PERFORMANCE_SUMMARY_FILE=""

if [[ "${RUN_ARTIFACTS_ENABLE}" == "1" ]]; then
    RUN_DIR="${RUNS_ROOT}/${RUN_TAG}"
    mkdir -p "${RUN_DIR}"

    if [[ -z "${BRIDGE_STATE_FILE}" ]]; then
        BRIDGE_STATE_FILE="${RUN_DIR}/detection_log.jsonl"
    fi
    if [[ -z "${HEALTH_STATE_FILE}" ]]; then
        HEALTH_STATE_FILE="${RUN_DIR}/health_log.jsonl"
    fi
    if [[ -z "${DEEPSTREAM_LOG_FILE}" ]]; then
        DEEPSTREAM_LOG_FILE="${RUN_DIR}/deepstream.log"
    fi
    if [[ "${RAW_RECORD_ENABLE}" == "1" && -z "${RAW_RECORD_FILE}" ]]; then
        RAW_RECORD_FILE="${RUN_DIR}/recording_clean.mkv"
    fi

    CONFIG_USED_FILE="${RUN_DIR}/config_used.yaml"
    PERFORMANCE_SUMMARY_FILE="${RUN_DIR}/performance_summary.txt"

    GIT_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
    GIT_BRANCH="$(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
    GIT_DIRTY_COUNT="$(git -C "${REPO_ROOT}" status --porcelain 2>/dev/null | wc -l | tr -d ' ')"
    [[ -z "${GIT_DIRTY_COUNT}" ]] && GIT_DIRTY_COUNT="0"
    NV_TEGRA_RELEASE="$(cat /etc/nv_tegra_release 2>/dev/null || echo unknown)"
    UNAME_TEXT="$(uname -a 2>/dev/null || echo unknown)"
    NVMODEL_TEXT="$(nvpmodel -q 2>/dev/null | tr '\n' ';' || echo unknown)"

    MODEL_ENGINE_FILE="$(infer_cfg_value "model-engine-file" "${INFER_CONFIG:-}")"
    ONNX_FILE="$(infer_cfg_value "onnx-file" "${INFER_CONFIG:-}")"
    LABEL_FILE="$(infer_cfg_value "labelfile-path" "${INFER_CONFIG:-}")"
    CONF_THRESHOLD="$(infer_cfg_value "pre-cluster-threshold" "${INFER_CONFIG:-}")"

    {
        echo "run_tag: $(yaml_quote "${RUN_TAG}")"
        echo "run_dir: $(yaml_quote "${RUN_DIR}")"
        echo "timestamp_local: $(yaml_quote "$(date '+%Y-%m-%d %H:%M:%S %z')")"
        echo "git_commit: $(yaml_quote "${GIT_COMMIT}")"
        echo "git_branch: $(yaml_quote "${GIT_BRANCH}")"
        echo "git_dirty_files: ${GIT_DIRTY_COUNT}"
        echo "jetson_info:"
        echo "  nv_tegra_release: $(yaml_quote "${NV_TEGRA_RELEASE}")"
        echo "  uname: $(yaml_quote "${UNAME_TEXT}")"
        echo "  nvpmodel: $(yaml_quote "${NVMODEL_TEXT}")"
        echo "camera:"
        echo "  sensor_id: ${SENSOR_ID}"
        echo "  width: ${CAMERA_WIDTH}"
        echo "  height: ${CAMERA_HEIGHT}"
        echo "  fps_n: ${CAMERA_FPS_N}"
        echo "  fps_d: ${CAMERA_FPS_D}"
        echo "model:"
        echo "  infer_config: $(yaml_quote "${INFER_CONFIG:-}")"
        echo "  model_engine_file: $(yaml_quote "${MODEL_ENGINE_FILE}")"
        echo "  onnx_file: $(yaml_quote "${ONNX_FILE}")"
        echo "  labels_file: $(yaml_quote "${LABEL_FILE}")"
        echo "  confidence_threshold: $(yaml_quote "${CONF_THRESHOLD}")"
        echo "tracker:"
        echo "  tracker_config: $(yaml_quote "${TRACKER_CONFIG:-}")"
        echo "selection:"
        echo "  target_class_id: ${TARGET_CLASS_ID}"
        echo "  selection: $(yaml_quote "${SELECTION}")"
        echo "  target_id: $(yaml_quote "${TARGET_ID}")"
        echo "control:"
        echo "  metadata_max_age_ms: ${METADATA_MAX_AGE_MS}"
        echo "  lost_buffer: ${LOST_BUFFER}"
        echo "  pan_gain: ${PAN_GAIN}"
        echo "  tilt_gain: ${TILT_GAIN}"
        echo "  deadzone: ${DEADZONE}"
        echo "  smooth_alpha: ${SMOOTH_ALPHA}"
        echo "  fast_smooth_alpha: ${FAST_SMOOTH_ALPHA}"
        echo "  fast_error_zone: ${FAST_ERROR_ZONE}"
        echo "  command_boost_zone: ${COMMAND_BOOST_ZONE}"
        echo "  min_active_command: ${MIN_ACTIVE_COMMAND}"
        echo "  response_gamma: ${RESPONSE_GAMMA}"
        echo "  pan_feedforward_gain: ${PAN_FEEDFORWARD_GAIN}"
        echo "  tilt_feedforward_gain: ${TILT_FEEDFORWARD_GAIN}"
        echo "  feedforward_alpha: ${FEEDFORWARD_ALPHA}"
        echo "  feedforward_limit: ${FEEDFORWARD_LIMIT}"
        echo "  feedforward_activation_zone: ${FEEDFORWARD_ACTIVATION_ZONE}"
        echo "  max_command: ${MAX_COMMAND}"
        echo "  send_rate_hz: ${SEND_RATE_HZ}"
        echo "  max_yaw_rate_dps: ${MAX_YAW_RATE_DPS}"
        echo "  max_pitch_rate_dps: ${MAX_PITCH_RATE_DPS}"
        echo "  initial_yaw_deg: ${INITIAL_YAW_DEG}"
        echo "  initial_pitch_deg: ${INITIAL_PITCH_DEG}"
        echo "  yaw_lock: ${YAW_LOCK}"
        echo "  pitch_lock: ${PITCH_LOCK}"
        echo "mavlink:"
        echo "  serial_device: $(yaml_quote "${SERIAL_DEVICE}")"
        echo "  serial_baud: ${SERIAL_BAUD}"
        echo "  target_system: ${MAV_TARGET_SYSTEM}"
        echo "  target_component: ${MAV_TARGET_COMPONENT}"
        echo "  gimbal_device_id: ${GIMBAL_DEVICE_ID}"
        echo "runtime:"
        echo "  show: ${SHOW}"
        echo "  rtsp_enable: ${RTSP_ENABLE}"
        echo "  rtsp_port: ${RTSP_PORT}"
        echo "  rtsp_mount: $(yaml_quote "${RTSP_MOUNT}")"
        echo "  raw_record_enable: ${RAW_RECORD_ENABLE}"
        echo "  raw_record_file: $(yaml_quote "${RAW_RECORD_FILE}")"
        echo "artifacts:"
        echo "  bridge_state_file: $(yaml_quote "${BRIDGE_STATE_FILE}")"
        echo "  health_state_file: $(yaml_quote "${HEALTH_STATE_FILE}")"
        echo "  deepstream_log_file: $(yaml_quote "${DEEPSTREAM_LOG_FILE}")"
        echo "  config_used_file: $(yaml_quote "${CONFIG_USED_FILE}")"
        echo "  performance_summary_file: $(yaml_quote "${PERFORMANCE_SUMMARY_FILE}")"
    } > "${CONFIG_USED_FILE}"

    ln -sfn "${RUN_DIR}" "${RUNS_ROOT}/latest"
fi

CMD=(
    "${PYTHON_BIN}"
    "${SCRIPT_DIR}/deepstream_px4_siyi_bridge.py"
    --serial-device "${SERIAL_DEVICE}"
    --serial-baud "${SERIAL_BAUD}"
    --mav-source-system "${MAV_SOURCE_SYSTEM}"
    --mav-source-component "${MAV_SOURCE_COMPONENT}"
    --mav-target-system "${MAV_TARGET_SYSTEM}"
    --mav-target-component "${MAV_TARGET_COMPONENT}"
    --gimbal-device-id "${GIMBAL_DEVICE_ID}"
    --heartbeat-timeout "${HEARTBEAT_TIMEOUT}"
    --ack-timeout "${ACK_TIMEOUT}"
    --send-rate-hz "${SEND_RATE_HZ}"
    --poll-interval "${POLL_INTERVAL}"
    --max-yaw-rate-dps "${MAX_YAW_RATE_DPS}"
    --max-pitch-rate-dps "${MAX_PITCH_RATE_DPS}"
    --live-control-mode "${LIVE_CONTROL_MODE}"
    --initial-yaw-deg "${INITIAL_YAW_DEG}"
    --initial-pitch-deg "${INITIAL_PITCH_DEG}"
    --max-yaw-angle-deg "${MAX_YAW_ANGLE_DEG}"
    --min-pitch-angle-deg "${MIN_PITCH_ANGLE_DEG}"
    --max-pitch-angle-deg "${MAX_PITCH_ANGLE_DEG}"
    --secondary-control-system "${SECONDARY_CONTROL_SYSTEM}"
    --secondary-control-component "${SECONDARY_CONTROL_COMPONENT}"
    --control-api "${CONTROL_API}"
)

if [[ -n "${METADATA_STATE_FILE}" ]]; then
    CMD+=(--metadata-state-file "${METADATA_STATE_FILE}")
fi
if [[ -n "${METADATA_IPC_FILE}" ]]; then
    CMD+=(--metadata-ipc-file "${METADATA_IPC_FILE}")
fi
if [[ -n "${BRIDGE_STATE_FILE}" ]]; then
    CMD+=(--bridge-state-file "${BRIDGE_STATE_FILE}")
fi
if [[ -n "${DEEPSTREAM_LOG_FILE}" ]]; then
    CMD+=(--deepstream-log-file "${DEEPSTREAM_LOG_FILE}")
fi
if [[ "${SHOW_DEEPSTREAM_LOG}" == "1" ]]; then
    CMD+=(--show-deepstream-log)
fi
if [[ "${YAW_LOCK}" == "1" ]]; then
    CMD+=(--yaw-lock)
fi
if [[ "${PITCH_LOCK}" == "1" ]]; then
    CMD+=(--pitch-lock)
fi
if [[ "${MAV_INVERT_PAN}" == "1" ]]; then
    CMD+=(--mav-invert-pan)
fi
if [[ "${MAV_INVERT_TILT}" == "1" ]]; then
    CMD+=(--mav-invert-tilt)
fi
if [[ "${SKIP_CONFIGURE}" == "1" ]]; then
    CMD+=(--skip-configure)
fi
if [[ "${DRY_RUN_MAVLINK}" == "1" ]]; then
    CMD+=(--dry-run-mavlink)
fi
if [[ "${PRINT_STATE}" == "1" ]]; then
    CMD+=(--print-state)
fi
if [[ "${PRINT_HEALTH}" == "1" ]]; then
    CMD+=(--print-health)
fi
CMD+=(--health-print-interval-sec "${HEALTH_PRINT_INTERVAL_SEC}")
if [[ -n "${HEALTH_STATE_FILE}" ]]; then
    CMD+=(--health-state-file "${HEALTH_STATE_FILE}")
fi
if [[ -n "${HEALTH_TEMP_ZONE}" ]]; then
    CMD+=(--health-temp-zone "${HEALTH_TEMP_ZONE}")
fi

export SENSOR_ID
export CAMERA_WIDTH
export CAMERA_HEIGHT
export CAMERA_FPS_N
export CAMERA_FPS_D
export METADATA_MAX_AGE_MS
export SHOW
export PRINT_FRAME_LOGS
export TARGET_CLASS_ID
export SELECTION
export TARGET_ID
export LOST_BUFFER
export PAN_GAIN
export TILT_GAIN
export DEADZONE
export SMOOTH_ALPHA
export FAST_SMOOTH_ALPHA
export FAST_ERROR_ZONE
export COMMAND_BOOST_ZONE
export MIN_ACTIVE_COMMAND
export RESPONSE_GAMMA
export PAN_FEEDFORWARD_GAIN
export TILT_FEEDFORWARD_GAIN
export FEEDFORWARD_ALPHA
export FEEDFORWARD_LIMIT
export FEEDFORWARD_ACTIVATION_ZONE
export MAX_COMMAND
export INFER_CONFIG
export TRACKER_CONFIG
export INVERT_PAN
export INVERT_TILT
export RTSP_ENABLE
export RTSP_PORT
export RTSP_MOUNT
export UDP_PORT
export BITRATE
export RAW_RECORD_ENABLE
export RAW_RECORD_FILE
export RAW_RECORD_BITRATE
export VIDEO_QUEUE_BUFFERS
export RTSP_QUEUE_BUFFERS
export RAW_RECORD_QUEUE_BUFFERS
export MAX_FRAMES
export STATE_FILE_FLUSH
export PRINT_HEALTH
export HEALTH_PRINT_INTERVAL_SEC
export HEALTH_STATE_FILE
export HEALTH_TEMP_ZONE
export RUN_ARTIFACTS_ENABLE
export RUNS_ROOT
export RUN_TAG

cd "${REPO_ROOT}"
if "${CMD[@]}" "$@"; then
    RUN_RC=0
else
    RUN_RC=$?
fi

if [[ "${RUN_ARTIFACTS_ENABLE}" == "1" && -n "${PERFORMANCE_SUMMARY_FILE}" ]]; then
    {
        echo "run_dir=${RUN_DIR}"
        echo "run_exit_code=${RUN_RC}"
        echo "bridge_state_file=${BRIDGE_STATE_FILE}"
        echo "health_state_file=${HEALTH_STATE_FILE}"
        echo "deepstream_log_file=${DEEPSTREAM_LOG_FILE}"
        [[ -n "${RAW_RECORD_FILE}" ]] && echo "raw_record_file=${RAW_RECORD_FILE}"
        echo
        if [[ -n "${BRIDGE_STATE_FILE}" && -f "${BRIDGE_STATE_FILE}" ]]; then
            python3 "${SCRIPT_DIR}/tools/bridge_latency_report.py" --bridge-log "${BRIDGE_STATE_FILE}" || true
        else
            echo "bridge_latency_report: bridge log not found"
        fi
    } > "${PERFORMANCE_SUMMARY_FILE}"
fi

exit "${RUN_RC}"
