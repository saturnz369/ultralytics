#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2"

export SENSOR_ID="${SENSOR_ID:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
export CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
export SHOW="${SHOW:-0}"
export RTSP_ENABLE=1
export RTSP_PORT="${RTSP_PORT:-8554}"
export UDP_PORT="${UDP_PORT:-5400}"

bash "${ROOT_DIR}/run_deepstream_yolo26_csi_app.sh"
