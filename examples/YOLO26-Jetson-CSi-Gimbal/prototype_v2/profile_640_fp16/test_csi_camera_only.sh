#!/usr/bin/env bash
set -euo pipefail

SENSOR_ID="${SENSOR_ID:-0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
NUM_BUFFERS="${NUM_BUFFERS:-60}"

gst-launch-1.0 -e \
  nvarguscamerasrc sensor-id="${SENSOR_ID}" num-buffers="${NUM_BUFFERS}" ! \
  "video/x-raw(memory:NVMM),width=${CAMERA_WIDTH},height=${CAMERA_HEIGHT},framerate=${CAMERA_FPS_N}/${CAMERA_FPS_D},format=NV12" ! \
  fakesink sync=false
