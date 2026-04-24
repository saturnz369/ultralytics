#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2"
BIN_PATH="${ROOT_DIR}/deepstream_yolo26_metadata_dump"

export SENSOR_ID="${SENSOR_ID:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
export CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
export SHOW="${SHOW:-1}"
export MAX_FRAMES="${MAX_FRAMES:-0}"
export INFER_CONFIG="${INFER_CONFIG:-${ROOT_DIR}/config/config_infer_primary_yolo26.txt}"
export STATE_FILE="${STATE_FILE:-/tmp/deepstream_yolo26_metadata.jsonl}"

if [[ ! -x "${BIN_PATH}" ]]; then
  make -C "${ROOT_DIR}"
fi

"${BIN_PATH}"
