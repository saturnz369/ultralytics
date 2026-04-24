#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"
MODEL_DIR="${ROOT_DIR}/model"
PYTHON_BIN="/home/aarl/DeepStream-Yolo/.venv-yolo26-sys/bin/python"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

WEIGHTS="${WEIGHTS:-/home/aarl/Downloads/yolo26n.pt}"
MODEL_BASENAME="${MODEL_BASENAME:-yolo26n_native_320}"
IMGSZ="${IMGSZ:-320}"
SOURCE_DIR="$(dirname "${WEIGHTS}")"
SOURCE_STEM="$(basename "${WEIGHTS%.pt}")"

mkdir -p "${MODEL_DIR}"

if [[ ! -f "${WEIGHTS}" ]]; then
  echo "Weights not found: ${WEIGHTS}" >&2
  exit 1
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python runtime not found: ${PYTHON_BIN}" >&2
  exit 1
fi

TMP_DIR="$(mktemp -d /tmp/prototype_v2_model_XXXXXX)"
trap 'rm -rf "${TMP_DIR}"' EXIT

(
  cd "${REPO_ROOT}"
  "${PYTHON_BIN}" - <<PY
from ultralytics import YOLO
model = YOLO("${WEIGHTS}")
out = model.export(format="onnx", imgsz=${IMGSZ}, opset=17, simplify=False, dynamic=False)
print(out)
PY
)

cp -f "${SOURCE_DIR}/${SOURCE_STEM}.onnx" "${MODEL_DIR}/${MODEL_BASENAME}.onnx"
if [[ -f "/home/aarl/DeepStream-Yolo/labels.txt" ]]; then
  cp -f "/home/aarl/DeepStream-Yolo/labels.txt" "${MODEL_DIR}/labels.txt"
fi
rm -f \
  "${MODEL_DIR}/${MODEL_BASENAME}_b1_gpu0_fp16.engine" \
  "${MODEL_DIR}/${MODEL_BASENAME}_b1_gpu0_fp32.engine"

echo "Prepared prototype_v2 model artifacts:"
echo "  ONNX:   ${MODEL_DIR}/${MODEL_BASENAME}.onnx"
echo "  Labels: ${MODEL_DIR}/labels.txt"
echo "DeepStream will rebuild the engine on the next run."
