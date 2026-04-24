#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"

PYTHON_BIN="${PYTHON_BIN:-/home/aarl/DeepStream-Yolo/.venv-yolo26-sys/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    PYTHON_BIN="python3"
fi

MODEL_PATH="${MODEL_PATH:-/home/aarl/DeepStream-Yolo/third_party/ultralytics/yolo26n.pt}"
SOURCE="${SOURCE:-csi}"
DEVICE="${DEVICE:-0}"
IMGSZ="${IMGSZ:-640}"
CONF="${CONF:-0.10}"
IOU="${IOU:-0.45}"
MAX_DET="${MAX_DET:-300}"
TRACKER="${TRACKER:-bytetrack.yaml}"
TRACK_CLASSES="${TRACK_CLASSES:-0}"
SELECTION="${SELECTION:-center}"
TARGET_ID="${TARGET_ID:-}"
LOST_BUFFER="${LOST_BUFFER:-15}"
SHOW_ALL_TRACKS="${SHOW_ALL_TRACKS:-0}"
PRINT_STATE="${PRINT_STATE:-0}"
STATE_FILE="${STATE_FILE:-}"

PAN_GAIN="${PAN_GAIN:-0.65}"
TILT_GAIN="${TILT_GAIN:-0.65}"
DEADZONE="${DEADZONE:-0.04}"
SMOOTH_ALPHA="${SMOOTH_ALPHA:-0.35}"
MAX_COMMAND="${MAX_COMMAND:-1.0}"
INVERT_PAN="${INVERT_PAN:-0}"
INVERT_TILT="${INVERT_TILT:-0}"

SERIAL_DEVICE="${SERIAL_DEVICE:-/dev/ttyUSB0}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"
MAV_SOURCE_SYSTEM="${MAV_SOURCE_SYSTEM:-42}"
MAV_SOURCE_COMPONENT="${MAV_SOURCE_COMPONENT:-191}"
MAV_TARGET_SYSTEM="${MAV_TARGET_SYSTEM:-1}"
MAV_TARGET_COMPONENT="${MAV_TARGET_COMPONENT:-154}"
GIMBAL_DEVICE_ID="${GIMBAL_DEVICE_ID:-154}"
HEARTBEAT_TIMEOUT="${HEARTBEAT_TIMEOUT:-5.0}"
ACK_TIMEOUT="${ACK_TIMEOUT:-2.0}"
SEND_RATE_HZ="${SEND_RATE_HZ:-20.0}"
MAX_YAW_RATE_DPS="${MAX_YAW_RATE_DPS:-60.0}"
MAX_PITCH_RATE_DPS="${MAX_PITCH_RATE_DPS:-45.0}"
LIVE_CONTROL_MODE="${LIVE_CONTROL_MODE:-angle-target}"
MAX_YAW_ANGLE_DEG="${MAX_YAW_ANGLE_DEG:-60.0}"
MIN_PITCH_ANGLE_DEG="${MIN_PITCH_ANGLE_DEG:--45.0}"
MAX_PITCH_ANGLE_DEG="${MAX_PITCH_ANGLE_DEG:-25.0}"
YAW_LOCK="${YAW_LOCK:-0}"
PITCH_LOCK="${PITCH_LOCK:-0}"
MAV_INVERT_PAN="${MAV_INVERT_PAN:-1}"
MAV_INVERT_TILT="${MAV_INVERT_TILT:-1}"
SKIP_CONFIGURE="${SKIP_CONFIGURE:-0}"
SECONDARY_CONTROL_SYSTEM="${SECONDARY_CONTROL_SYSTEM:--1}"
SECONDARY_CONTROL_COMPONENT="${SECONDARY_CONTROL_COMPONENT:--1}"
DRY_RUN_MAVLINK="${DRY_RUN_MAVLINK:-0}"
CONTROL_API="${CONTROL_API:-attitude}"

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
CAPTURE_WIDTH="${CAPTURE_WIDTH:-1280}"
CAPTURE_HEIGHT="${CAPTURE_HEIGHT:-720}"
DISPLAY_WIDTH="${DISPLAY_WIDTH:-1280}"
DISPLAY_HEIGHT="${DISPLAY_HEIGHT:-720}"
FRAMERATE="${FRAMERATE:-30}"
FLIP_METHOD="${FLIP_METHOD:-0}"

SHOW="${SHOW:-1}"
SAVE="${SAVE:-0}"
HALF="${HALF:-}"
OUTPUT="${OUTPUT:-}"
MAX_FRAMES="${MAX_FRAMES:-0}"

if [[ -z "${HALF}" ]]; then
    if [[ "${DEVICE}" == "cpu" || "${DEVICE}" == "mps" ]]; then
        HALF="0"
    else
        HALF="1"
    fi
fi

CMD=(
    "${PYTHON_BIN}"
    "${SCRIPT_DIR}/jetson_csi_px4_siyi_bridge.py"
    --model "${MODEL_PATH}"
    --source "${SOURCE}"
    --device "${DEVICE}"
    --imgsz "${IMGSZ}"
    --conf "${CONF}"
    --iou "${IOU}"
    --max-det "${MAX_DET}"
    --tracker "${TRACKER}"
    --selection "${SELECTION}"
    --lost-buffer "${LOST_BUFFER}"
    --pan-gain "${PAN_GAIN}"
    --tilt-gain "${TILT_GAIN}"
    --deadzone "${DEADZONE}"
    --smooth-alpha "${SMOOTH_ALPHA}"
    --max-command "${MAX_COMMAND}"
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
    --max-yaw-rate-dps "${MAX_YAW_RATE_DPS}"
    --max-pitch-rate-dps "${MAX_PITCH_RATE_DPS}"
    --live-control-mode "${LIVE_CONTROL_MODE}"
    --max-yaw-angle-deg "${MAX_YAW_ANGLE_DEG}"
    --min-pitch-angle-deg "${MIN_PITCH_ANGLE_DEG}"
    --max-pitch-angle-deg "${MAX_PITCH_ANGLE_DEG}"
    --secondary-control-system "${SECONDARY_CONTROL_SYSTEM}"
    --secondary-control-component "${SECONDARY_CONTROL_COMPONENT}"
    --control-api "${CONTROL_API}"
    --sensor-id "${SENSOR_ID}"
    --capture-width "${CAPTURE_WIDTH}"
    --capture-height "${CAPTURE_HEIGHT}"
    --display-width "${DISPLAY_WIDTH}"
    --display-height "${DISPLAY_HEIGHT}"
    --framerate "${FRAMERATE}"
    --flip-method "${FLIP_METHOD}"
    --max-frames "${MAX_FRAMES}"
)

if [[ "${SHOW}" == "1" ]]; then
    CMD+=(--show)
fi

if [[ "${SAVE}" == "1" ]]; then
    CMD+=(--save)
fi

if [[ "${HALF}" == "1" ]]; then
    CMD+=(--half)
fi

if [[ "${SHOW_ALL_TRACKS}" == "1" ]]; then
    CMD+=(--show-all-tracks)
fi

if [[ "${PRINT_STATE}" == "1" ]]; then
    CMD+=(--print-state)
fi

if [[ "${INVERT_PAN}" == "1" ]]; then
    CMD+=(--invert-pan)
fi

if [[ "${INVERT_TILT}" == "1" ]]; then
    CMD+=(--invert-tilt)
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

if [[ -n "${TRACK_CLASSES}" ]]; then
    # shellcheck disable=SC2206
    CLASS_LIST=(${TRACK_CLASSES})
    CMD+=(--classes "${CLASS_LIST[@]}")
fi

if [[ -n "${TARGET_ID}" ]]; then
    CMD+=(--target-id "${TARGET_ID}")
fi

if [[ -n "${STATE_FILE}" ]]; then
    CMD+=(--state-file "${STATE_FILE}")
fi

if [[ -n "${OUTPUT}" ]]; then
    CMD+=(--output "${OUTPUT}")
fi

cd "${REPO_ROOT}"
exec "${CMD[@]}" "$@"
