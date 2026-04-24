#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/aarl/ultralytics/examples/YOLO26-Jetson-CSI-Inference/prototype_v2"
SERVER_SCRIPT="${ROOT_DIR}/run_deepstream_yolo26_rtsp_target_control.sh"

export RTSP_PORT="${RTSP_PORT:-8554}"
export RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:${RTSP_PORT}/ds-test}"
export PREVIEW_WAIT_SECONDS="${PREVIEW_WAIT_SECONDS:-15}"
export RTSP_VIEWER_LATENCY_MS="${RTSP_VIEWER_LATENCY_MS:-80}"
export RTSP_PROTOCOLS="${RTSP_PROTOCOLS:-udp}"

server_pid=""

cleanup() {
  if [[ -n "${server_pid}" ]] && kill -0 "${server_pid}" 2>/dev/null; then
    kill "${server_pid}" 2>/dev/null || true
    wait "${server_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

bash "${SERVER_SCRIPT}" &
server_pid=$!

deadline=$((SECONDS + PREVIEW_WAIT_SECONDS))
until ss -ltn | rg -q ":${RTSP_PORT}\\b"; do
  if ! kill -0 "${server_pid}" 2>/dev/null; then
    echo "DeepStream combined RTSP server exited before preview was ready." >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for RTSP server on port ${RTSP_PORT}." >&2
    exit 1
  fi
  sleep 1
done

echo "RTSP stream ready at ${RTSP_URL}"
gst-launch-1.0 -e \
  rtspsrc location="${RTSP_URL}" latency="${RTSP_VIEWER_LATENCY_MS}" drop-on-latency=true buffer-mode=none protocols="${RTSP_PROTOCOLS}" ! \
  rtph264depay ! h264parse ! \
  nvv4l2decoder enable-max-performance=true disable-dpb=true ! \
  queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! \
  nvvidconv ! \
  autovideosink sync=false async=false
