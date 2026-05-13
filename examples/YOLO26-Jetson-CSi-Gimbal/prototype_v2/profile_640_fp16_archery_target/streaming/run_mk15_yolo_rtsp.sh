#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

export SHOW="${SHOW:-0}"
export RTSP_ENABLE="${RTSP_ENABLE:-1}"
export RTSP_PORT="${RTSP_PORT:-8554}"
export RTSP_MOUNT="${RTSP_MOUNT:-/stream}"
export SENSOR_ID="${SENSOR_ID:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-2028}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-1112}"
export CAMERA_FPS_N="${CAMERA_FPS_N:-60}"
export CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
export TARGET_CLASS_ID="${TARGET_CLASS_ID:-0}"
export SELECTION="${SELECTION:-center}"
export MAX_FRAMES="${MAX_FRAMES:-0}"

exec bash "${PROFILE_DIR}/run_deepstream_yolo26_rtsp_target_control.sh"
