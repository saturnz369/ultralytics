#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RECORDINGS_DIR="${PROFILE_DIR}/recordings"
RUNS_DIR="${PROFILE_DIR}/runs"

if [[ -z "${VIDEO:-}" ]]; then
  if [[ -f "${RUNS_DIR}/latest/recording_clean.mkv" ]]; then
    VIDEO="${RUNS_DIR}/latest/recording_clean.mkv"
  else
    VIDEO="$(find "${RECORDINGS_DIR}" -maxdepth 1 -type f -name 'archery_target_clean_*.mkv' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
  fi
fi

if [[ -z "${VIDEO:-}" ]]; then
  echo "No clean recording found under ${RUNS_DIR}/latest or ${RECORDINGS_DIR}" >&2
  echo "Run the flight/test launch with RAW_RECORD_ENABLE=1 first." >&2
  exit 1
fi

if [[ -z "${BRIDGE_LOG:-}" ]]; then
  if [[ -f "${RUNS_DIR}/latest/detection_log.jsonl" ]]; then
    BRIDGE_LOG="${RUNS_DIR}/latest/detection_log.jsonl"
  else
    BRIDGE_LOG="$(find /tmp -maxdepth 1 -type f \( \
        -name 'profile640fp16_archery_target_mk15_live_bridge.jsonl' -o \
        -name 'profile640fp16_archery_target_live_bridge.jsonl' -o \
        -name 'profile640fp16_archery_target_bridge_dryrun.jsonl' \
      \) -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-)"
  fi
fi

if [[ -z "${BRIDGE_LOG:-}" ]]; then
  echo "No bridge JSONL log found in ${RUNS_DIR}/latest or /tmp" >&2
  exit 1
fi

MIN_MISS_FRAMES="${MIN_MISS_FRAMES:-6}"
MAX_MISS_FRAMES="${MAX_MISS_FRAMES:-600}"
FRAMES_PER_SEGMENT="${FRAMES_PER_SEGMENT:-4}"
INCLUDE_EDGE_MISSES="${INCLUDE_EDGE_MISSES:-1}"

CMD=(
  python3
  "${SCRIPT_DIR}/extract_failure_frames.py"
  --bridge-log "${BRIDGE_LOG}"
  --video "${VIDEO}"
  --min-miss-frames "${MIN_MISS_FRAMES}"
  --max-miss-frames "${MAX_MISS_FRAMES}"
  --frames-per-segment "${FRAMES_PER_SEGMENT}"
)

if [[ -n "${OUTPUT_ROOT:-}" ]]; then
  CMD+=(--output-root "${OUTPUT_ROOT}")
fi
if [[ "${INCLUDE_EDGE_MISSES}" == "1" ]]; then
  CMD+=(--include-edge-misses)
fi
if [[ -n "${SESSION_INDEX:-}" ]]; then
  CMD+=(--session-index "${SESSION_INDEX}")
fi

echo "video=${VIDEO}"
echo "bridge_log=${BRIDGE_LOG}"
echo "min_miss_frames=${MIN_MISS_FRAMES}"
echo "max_miss_frames=${MAX_MISS_FRAMES}"
echo "frames_per_segment=${FRAMES_PER_SEGMENT}"
exec "${CMD[@]}"
