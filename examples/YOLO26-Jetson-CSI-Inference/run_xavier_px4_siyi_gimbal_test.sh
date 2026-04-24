#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"

PYTHON_BIN="${PYTHON_BIN:-/home/aarl/DeepStream-Yolo/.venv-yolo26-sys/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    PYTHON_BIN="python3"
fi

MODE="${MODE:-angle}"
PITCH="${PITCH:-0}"
YAW="${YAW:-0}"
PITCH_RATE="${PITCH_RATE:-0}"
YAW_RATE="${YAW_RATE:-0}"
HOLD_SECONDS="${HOLD_SECONDS:-2.0}"
RESEND_HZ="${RESEND_HZ:-20.0}"
NEUTRAL_AFTER="${NEUTRAL_AFTER:-0}"
NEUTRAL_PITCH="${NEUTRAL_PITCH:-0}"
NEUTRAL_YAW="${NEUTRAL_YAW:-0}"
NEUTRAL_HOLD_SECONDS="${NEUTRAL_HOLD_SECONDS:-1.0}"

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
CONTROL_API="${CONTROL_API:-attitude}"
YAW_LOCK="${YAW_LOCK:-0}"
PITCH_LOCK="${PITCH_LOCK:-0}"
SKIP_CONFIGURE="${SKIP_CONFIGURE:-0}"
SECONDARY_CONTROL_SYSTEM="${SECONDARY_CONTROL_SYSTEM:--1}"
SECONDARY_CONTROL_COMPONENT="${SECONDARY_CONTROL_COMPONENT:--1}"
DRY_RUN_MAVLINK="${DRY_RUN_MAVLINK:-0}"

if [[ "${THORNG_DIALOUT_REEXEC:-0}" != "1" ]] && [[ -n "${SERIAL_DEVICE}" ]] && [[ -e "${SERIAL_DEVICE}" ]]; then
    if [[ ! -r "${SERIAL_DEVICE}" || ! -w "${SERIAL_DEVICE}" ]]; then
        if command -v sg >/dev/null 2>&1 && getent group dialout | grep -qw "$(id -un)"; then
            printf -v SELF_CMD '%q ' "$0" "$@"
            echo "Re-launching under dialout group for ${SERIAL_DEVICE} access..."
            exec sg dialout -c "THORNG_DIALOUT_REEXEC=1 bash ${SELF_CMD}"
        fi
    fi
fi

CMD=(
    "${PYTHON_BIN}"
    "${SCRIPT_DIR}/jetson_px4_siyi_gimbal_test.py"
    --mode "${MODE}"
    --pitch "${PITCH}"
    --yaw "${YAW}"
    --pitch-rate "${PITCH_RATE}"
    --yaw-rate "${YAW_RATE}"
    --hold-seconds "${HOLD_SECONDS}"
    --resend-hz "${RESEND_HZ}"
    --neutral-pitch "${NEUTRAL_PITCH}"
    --neutral-yaw "${NEUTRAL_YAW}"
    --neutral-hold-seconds "${NEUTRAL_HOLD_SECONDS}"
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
    --control-api "${CONTROL_API}"
    --secondary-control-system "${SECONDARY_CONTROL_SYSTEM}"
    --secondary-control-component "${SECONDARY_CONTROL_COMPONENT}"
)

if [[ "${NEUTRAL_AFTER}" == "1" ]]; then
    CMD+=(--neutral-after)
fi

if [[ "${YAW_LOCK}" == "1" ]]; then
    CMD+=(--yaw-lock)
fi

if [[ "${PITCH_LOCK}" == "1" ]]; then
    CMD+=(--pitch-lock)
fi

if [[ "${SKIP_CONFIGURE}" == "1" ]]; then
    CMD+=(--skip-configure)
fi

if [[ "${DRY_RUN_MAVLINK}" == "1" ]]; then
    CMD+=(--dry-run-mavlink)
fi

cd "${REPO_ROOT}"
exec "${CMD[@]}" "$@"
