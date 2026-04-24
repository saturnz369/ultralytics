#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"

PYTHON_BIN="${PYTHON_BIN:-/home/aarl/DeepStream-Yolo/.venv-yolo26-sys/bin/python}"
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
CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
SHOW="${SHOW:-0}"
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
UDP_PORT="${UDP_PORT:-5400}"
BITRATE="${BITRATE:-4000000}"
MAX_FRAMES="${MAX_FRAMES:-0}"
METADATA_STATE_FILE="${METADATA_STATE_FILE:-}"
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
MAX_YAW_RATE_DPS="${MAX_YAW_RATE_DPS:-95.0}"
MAX_PITCH_RATE_DPS="${MAX_PITCH_RATE_DPS:-60.0}"
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
PRINT_STATE="${PRINT_STATE:-0}"
BRIDGE_STATE_FILE="${BRIDGE_STATE_FILE:-}"

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
    --max-yaw-rate-dps "${MAX_YAW_RATE_DPS}"
    --max-pitch-rate-dps "${MAX_PITCH_RATE_DPS}"
    --live-control-mode "${LIVE_CONTROL_MODE}"
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

export SENSOR_ID
export CAMERA_WIDTH
export CAMERA_HEIGHT
export CAMERA_FPS_N
export CAMERA_FPS_D
export SHOW
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
export INVERT_PAN
export INVERT_TILT
export RTSP_ENABLE
export RTSP_PORT
export UDP_PORT
export BITRATE
export MAX_FRAMES

cd "${REPO_ROOT}"
exec "${CMD[@]}" "$@"
