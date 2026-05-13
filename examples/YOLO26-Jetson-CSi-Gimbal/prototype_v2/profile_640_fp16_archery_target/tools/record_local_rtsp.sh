#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/stream}"
OUTPUT_DIR="${OUTPUT_DIR:-${PROFILE_DIR}/recordings}"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
OUTPUT_FILE="${OUTPUT_FILE:-${OUTPUT_DIR}/archery_target_rtsp_${TIMESTAMP}.mkv}"

mkdir -p "${OUTPUT_DIR}"

echo "Recording RTSP stream:"
echo "  URL:    ${RTSP_URL}"
echo "  Output: ${OUTPUT_FILE}"
echo
echo "Stop with 'q' in the ffmpeg terminal or Ctrl+C."
echo

exec ffmpeg \
  -rtsp_transport tcp \
  -i "${RTSP_URL}" \
  -c copy \
  -map 0:v:0 \
  -f matroska \
  "${OUTPUT_FILE}"
