#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"
BIN_PATH="${ROOT_DIR}/deepstream_yolo26_rtsp_target_control"

export SENSOR_ID="${SENSOR_ID:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-2028}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-1112}"
export CAMERA_FPS_N="${CAMERA_FPS_N:-60}"
export CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
export SHOW="${SHOW:-0}"
export PRINT_FRAME_LOGS="${PRINT_FRAME_LOGS:-0}"
export ALLOW_MAX_FRAMES="${ALLOW_MAX_FRAMES:-0}"
if [[ "${ALLOW_MAX_FRAMES}" == "1" ]]; then
  export MAX_FRAMES="${MAX_FRAMES:-0}"
else
  if [[ -n "${MAX_FRAMES+x}" && "${MAX_FRAMES}" != "0" ]]; then
    echo "Ignoring inherited MAX_FRAMES=${MAX_FRAMES}; forcing unlimited run. Set ALLOW_MAX_FRAMES=1 to use bounded runs."
  fi
  export MAX_FRAMES=0
fi
export TARGET_CLASS_ID="${TARGET_CLASS_ID:-0}"
export SELECTION="${SELECTION:-center}"
export TARGET_ID="${TARGET_ID:--1}"
export LOST_BUFFER="${LOST_BUFFER:-15}"
export PAN_GAIN="${PAN_GAIN:-0.91}"
export TILT_GAIN="${TILT_GAIN:-0.83}"
export DEADZONE="${DEADZONE:-0.048}"
export SMOOTH_ALPHA="${SMOOTH_ALPHA:-0.39}"
export FAST_SMOOTH_ALPHA="${FAST_SMOOTH_ALPHA:-0.81}"
export FAST_ERROR_ZONE="${FAST_ERROR_ZONE:-0.15}"
export COMMAND_BOOST_ZONE="${COMMAND_BOOST_ZONE:-0.095}"
export MIN_ACTIVE_COMMAND="${MIN_ACTIVE_COMMAND:-0.20}"
export RESPONSE_GAMMA="${RESPONSE_GAMMA:-0.63}"
export PAN_FEEDFORWARD_GAIN="${PAN_FEEDFORWARD_GAIN:-0.18}"
export TILT_FEEDFORWARD_GAIN="${TILT_FEEDFORWARD_GAIN:-0.12}"
export FEEDFORWARD_ALPHA="${FEEDFORWARD_ALPHA:-0.40}"
export FEEDFORWARD_LIMIT="${FEEDFORWARD_LIMIT:-0.14}"
export FEEDFORWARD_ACTIVATION_ZONE="${FEEDFORWARD_ACTIVATION_ZONE:-0.07}"
export MAX_COMMAND="${MAX_COMMAND:-1.0}"
export INFER_CONFIG="${INFER_CONFIG:-${ROOT_DIR}/config/config_infer_primary_yolo26.txt}"
export TRACKER_CONFIG="${TRACKER_CONFIG:-${ROOT_DIR}/config/tracker_config.txt}"
export STATE_FILE="${STATE_FILE:-}"
export STATE_FILE_FLUSH="${STATE_FILE_FLUSH:-0}"
export RTSP_ENABLE="${RTSP_ENABLE:-1}"
export RTSP_PORT="${RTSP_PORT:-8554}"
export UDP_PORT="${UDP_PORT:-5400}"
export BITRATE="${BITRATE:-8000000}"
export VIDEO_QUEUE_BUFFERS="${VIDEO_QUEUE_BUFFERS:-2}"

if [[ ! -x "${BIN_PATH}" ]]; then
  make -C "${ROOT_DIR}"
fi

cd "${ROOT_DIR}/model"
"${BIN_PATH}"
