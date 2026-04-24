#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH:-}"

PYTHON_BIN="${PYTHON_BIN:-/home/aarl/DeepStream-Yolo/.venv-yolo26-sys/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    PYTHON_BIN="python3"
fi

START_PITCH="${START_PITCH:-0}"
START_YAW="${START_YAW:-0}"
TARGET_PITCH="${TARGET_PITCH:-0}"
TARGET_YAW="${TARGET_YAW:-0}"
SECOND_TARGET_PITCH="${SECOND_TARGET_PITCH:-nan}"
SECOND_TARGET_YAW="${SECOND_TARGET_YAW:-nan}"
THIRD_TARGET_PITCH="${THIRD_TARGET_PITCH:-nan}"
THIRD_TARGET_YAW="${THIRD_TARGET_YAW:-nan}"
START_HOLD_SECONDS="${START_HOLD_SECONDS:-1.5}"
TARGET_HOLD_SECONDS="${TARGET_HOLD_SECONDS:-4.0}"
SECOND_TARGET_HOLD_SECONDS="${SECOND_TARGET_HOLD_SECONDS:-0.0}"
THIRD_TARGET_HOLD_SECONDS="${THIRD_TARGET_HOLD_SECONDS:-0.0}"
MOVE_TRANSIT_SECONDS="${MOVE_TRANSIT_SECONDS:-0.0}"
START_TRANSIT_SECONDS="${START_TRANSIT_SECONDS:-nan}"
TARGET_TRANSIT_SECONDS="${TARGET_TRANSIT_SECONDS:-nan}"
SECOND_TARGET_TRANSIT_SECONDS="${SECOND_TARGET_TRANSIT_SECONDS:-nan}"
THIRD_TARGET_TRANSIT_SECONDS="${THIRD_TARGET_TRANSIT_SECONDS:-nan}"
RETURN_TRANSIT_SECONDS="${RETURN_TRANSIT_SECONDS:-nan}"
MOVE_PITCH_RATE_DPS="${MOVE_PITCH_RATE_DPS:-nan}"
MOVE_YAW_RATE_DPS="${MOVE_YAW_RATE_DPS:-nan}"
RESEND_HZ="${RESEND_HZ:-20.0}"
SAMPLE_HZ="${SAMPLE_HZ:-20.0}"
FEEDBACK_REQUEST_HZ="${FEEDBACK_REQUEST_HZ:-10.0}"
SETTLE_THRESHOLD_DEG="${SETTLE_THRESHOLD_DEG:-3.0}"
STATE_FILE="${STATE_FILE:-/tmp/gimbal_speed_step_test.jsonl}"
RETURN_AFTER="${RETURN_AFTER:-0}"
RETURN_HOLD_SECONDS="${RETURN_HOLD_SECONDS:-1.5}"

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
    "${SCRIPT_DIR}/jetson_px4_siyi_gimbal_speed_test.py"
    --start-pitch "${START_PITCH}"
    --start-yaw "${START_YAW}"
    --target-pitch "${TARGET_PITCH}"
    --target-yaw "${TARGET_YAW}"
    --second-target-pitch "${SECOND_TARGET_PITCH}"
    --second-target-yaw "${SECOND_TARGET_YAW}"
    --third-target-pitch "${THIRD_TARGET_PITCH}"
    --third-target-yaw "${THIRD_TARGET_YAW}"
    --start-hold-seconds "${START_HOLD_SECONDS}"
    --target-hold-seconds "${TARGET_HOLD_SECONDS}"
    --second-target-hold-seconds "${SECOND_TARGET_HOLD_SECONDS}"
    --third-target-hold-seconds "${THIRD_TARGET_HOLD_SECONDS}"
    --move-transit-seconds "${MOVE_TRANSIT_SECONDS}"
    --start-transit-seconds "${START_TRANSIT_SECONDS}"
    --target-transit-seconds "${TARGET_TRANSIT_SECONDS}"
    --second-target-transit-seconds "${SECOND_TARGET_TRANSIT_SECONDS}"
    --third-target-transit-seconds "${THIRD_TARGET_TRANSIT_SECONDS}"
    --return-transit-seconds "${RETURN_TRANSIT_SECONDS}"
    --move-pitch-rate-dps "${MOVE_PITCH_RATE_DPS}"
    --move-yaw-rate-dps "${MOVE_YAW_RATE_DPS}"
    --resend-hz "${RESEND_HZ}"
    --sample-hz "${SAMPLE_HZ}"
    --feedback-request-hz "${FEEDBACK_REQUEST_HZ}"
    --settle-threshold-deg "${SETTLE_THRESHOLD_DEG}"
    --state-file "${STATE_FILE}"
    --return-hold-seconds "${RETURN_HOLD_SECONDS}"
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

if [[ "${RETURN_AFTER}" == "1" ]]; then
    CMD+=(--return-after)
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
