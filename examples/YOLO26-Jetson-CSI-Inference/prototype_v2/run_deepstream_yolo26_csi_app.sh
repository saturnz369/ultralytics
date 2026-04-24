#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2"
CONFIG_DIR="${ROOT_DIR}/config"
TMP_CONFIG="$(mktemp /tmp/prototype_v2_ds_app_XXXXXX.txt)"
trap 'rm -f "${TMP_CONFIG}"' EXIT

SENSOR_ID="${SENSOR_ID:-0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
CAMERA_FPS_N="${CAMERA_FPS_N:-30}"
CAMERA_FPS_D="${CAMERA_FPS_D:-1}"
SHOW="${SHOW:-1}"
RTSP_ENABLE="${RTSP_ENABLE:-0}"
RTSP_PORT="${RTSP_PORT:-8554}"
UDP_PORT="${UDP_PORT:-5400}"
INFER_CONFIG="${INFER_CONFIG:-${CONFIG_DIR}/config_infer_primary_yolo26.txt}"
APP_TEMPLATE="${APP_TEMPLATE:-${CONFIG_DIR}/deepstream_app_yolo26_csi.template.txt}"

sed \
  -e "s|__SENSOR_ID__|${SENSOR_ID}|g" \
  -e "s|__CAMERA_WIDTH__|${CAMERA_WIDTH}|g" \
  -e "s|__CAMERA_HEIGHT__|${CAMERA_HEIGHT}|g" \
  -e "s|__CAMERA_FPS_N__|${CAMERA_FPS_N}|g" \
  -e "s|__CAMERA_FPS_D__|${CAMERA_FPS_D}|g" \
  -e "s|__SHOW_ENABLE__|${SHOW}|g" \
  -e "s|__RTSP_ENABLE__|${RTSP_ENABLE}|g" \
  -e "s|__RTSP_PORT__|${RTSP_PORT}|g" \
  -e "s|__UDP_PORT__|${UDP_PORT}|g" \
  -e "s|__INFER_CONFIG__|${INFER_CONFIG}|g" \
  "${APP_TEMPLATE}" > "${TMP_CONFIG}"

deepstream-app -c "${TMP_CONFIG}"
