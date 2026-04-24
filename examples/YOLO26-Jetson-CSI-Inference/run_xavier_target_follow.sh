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
TRACK_CLASSES="${TRACK_CLASSES:-}"
SELECTION="${SELECTION:-center}"
TARGET_ID="${TARGET_ID:-}"
LOST_BUFFER="${LOST_BUFFER:-15}"
SHOW_ALL_TRACKS="${SHOW_ALL_TRACKS:-0}"
PRINT_STATE="${PRINT_STATE:-0}"
STATE_FILE="${STATE_FILE:-}"

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
    "${SCRIPT_DIR}/jetson_csi_target_follow.py"
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
