#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"
BIN_PATH="${ROOT_DIR}/deepstream_yolo26_rtsp_target_control"

export SENSOR_ID="${SENSOR_ID:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
export CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
export SHOW="${SHOW:-0}"
export MAX_FRAMES="${MAX_FRAMES:-0}"
export TARGET_CLASS_ID="${TARGET_CLASS_ID:-0}"
export SELECTION="${SELECTION:-center}"
export TARGET_ID="${TARGET_ID:--1}"
export LOST_BUFFER="${LOST_BUFFER:-15}"
export PAN_GAIN="${PAN_GAIN:-0.85}"
export TILT_GAIN="${TILT_GAIN:-0.80}"
export DEADZONE="${DEADZONE:-0.05}"
export SMOOTH_ALPHA="${SMOOTH_ALPHA:-0.40}"
export FAST_SMOOTH_ALPHA="${FAST_SMOOTH_ALPHA:-0.75}"
export FAST_ERROR_ZONE="${FAST_ERROR_ZONE:-0.18}"
export COMMAND_BOOST_ZONE="${COMMAND_BOOST_ZONE:-0.10}"
export MIN_ACTIVE_COMMAND="${MIN_ACTIVE_COMMAND:-0.18}"
export RESPONSE_GAMMA="${RESPONSE_GAMMA:-0.65}"
export MAX_COMMAND="${MAX_COMMAND:-1.0}"
export INFER_CONFIG="${INFER_CONFIG:-${ROOT_DIR}/config/config_infer_primary_yolo26.txt}"
export TRACKER_CONFIG="${TRACKER_CONFIG:-${ROOT_DIR}/config/tracker_config.txt}"
export STATE_FILE="${STATE_FILE:-/tmp/profile640_deepstream_yolo26_rtsp_target_control.jsonl}"
export RTSP_ENABLE="${RTSP_ENABLE:-1}"
export RTSP_PORT="${RTSP_PORT:-8554}"
export UDP_PORT="${UDP_PORT:-5400}"
export BITRATE="${BITRATE:-4000000}"

if [[ ! -x "${BIN_PATH}" ]]; then
  make -C "${ROOT_DIR}"
fi

cd "${ROOT_DIR}/model"
"${BIN_PATH}"
