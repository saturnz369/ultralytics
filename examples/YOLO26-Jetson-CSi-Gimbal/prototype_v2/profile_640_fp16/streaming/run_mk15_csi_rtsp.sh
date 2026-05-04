#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN_PATH="${SCRIPT_DIR}/csi_h264_rtsp_server"

export SENSOR_ID="${SENSOR_ID:-0}"
export CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
export CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
export CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
export CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
export VIDEO_SOURCE="${VIDEO_SOURCE:-csi}"
export RTSP_HOST_IP="${RTSP_HOST_IP:-192.168.144.100}"
export RTSP_PORT="${RTSP_PORT:-8554}"
export RTSP_MOUNT="${RTSP_MOUNT:-/stream}"
export RTSP_ALIAS_MOUNT="${RTSP_ALIAS_MOUNT:-/main.264}"
export BITRATE="${BITRATE:-3000000}"
export IFRAME_INTERVAL="${IFRAME_INTERVAL:-30}"

if [[ ! -x "${BIN_PATH}" ]]; then
  make -C "${PROFILE_DIR}" streaming/csi_h264_rtsp_server
fi

exec "${BIN_PATH}"
