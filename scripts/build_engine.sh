#!/usr/bin/env bash
# Build TensorRT engine files from ONNX or WTS model weights.
#
# Supports two workflows:
#   1. ONNX (recommended): PyTorch .pt -> ONNX -> TensorRT engine
#   2. WTS (legacy):       PyTorch .pt -> .wts -> custom builder -> engine
#
# Usage:
#   ./scripts/build_engine.sh onnx model.onnx              # ONNX to engine
#   ./scripts/build_engine.sh onnx model.onnx --fp16        # ONNX with FP16
#   ./scripts/build_engine.sh onnx model.onnx --int8 calib/ # ONNX with INT8
#   ./scripts/build_engine.sh export model.pt               # PyTorch to ONNX
#   ./scripts/build_engine.sh info engine.engine             # Print engine info
#
# Requirements:
#   ONNX path: trtexec (included with TensorRT), python3 + ultralytics
#   WTS path:  custom builder binary (build with cmake in tools/engine_builder/)
#
# Output: <model_name>.engine in the same directory as the input file.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Default engine build parameters (matching detector/config.hpp)
INPUT_H=480
INPUT_W=640
BATCH_SIZE=1
WORKSPACE_MB=1024

usage() {
  echo "Usage:"
  echo "  $0 onnx <model.onnx> [--fp16|--int8 <calib_dir>] [--output <out.engine>]"
  echo "  $0 export <model.pt> [--imgsz 640 480]"
  echo "  $0 info <engine.engine>"
  echo ""
  echo "Environment variables:"
  echo "  TRTEXEC    Path to trtexec binary (auto-detected if not set)"
  echo "  INPUT_H    Input height (default: $INPUT_H)"
  echo "  INPUT_W    Input width (default: $INPUT_W)"
  exit 1
}

find_trtexec() {
  if [ -n "${TRTEXEC:-}" ] && [ -x "$TRTEXEC" ]; then
    echo "$TRTEXEC"
    return
  fi
  # Common locations
  for path in \
    /usr/src/tensorrt/bin/trtexec \
    /usr/local/bin/trtexec \
    /usr/bin/trtexec \
    "$HOME/TensorRT*/bin/trtexec" \
    /opt/nvidia/tensorrt/bin/trtexec; do
    # shellcheck disable=SC2086
    for p in $path; do
      if [ -x "$p" ]; then
        echo "$p"
        return
      fi
    done
  done
  echo ""
}

cmd_onnx() {
  local onnx_file="$1"; shift
  local precision="--fp32"
  local calib_dir=""
  local output=""

  while [ $# -gt 0 ]; do
    case "$1" in
      --fp16) precision="--fp16"; shift ;;
      --int8) precision="--int8"; calib_dir="$2"; shift 2 ;;
      --output) output="$2"; shift 2 ;;
      *) echo "Unknown option: $1"; usage ;;
    esac
  done

  if [ ! -f "$onnx_file" ]; then
    echo "Error: ONNX file not found: $onnx_file"
    exit 1
  fi

  local trtexec
  trtexec="$(find_trtexec)"
  if [ -z "$trtexec" ]; then
    echo "Error: trtexec not found. Install TensorRT or set TRTEXEC env var."
    exit 1
  fi

  if [ -z "$output" ]; then
    output="${onnx_file%.onnx}.engine"
  fi

  echo "Building TensorRT engine:"
  echo "  Input:     $onnx_file"
  echo "  Output:    $output"
  echo "  Precision: $precision"
  echo "  Shape:     ${BATCH_SIZE}x3x${INPUT_H}x${INPUT_W}"
  echo "  Workspace: ${WORKSPACE_MB}MB"
  echo ""

  local args=(
    --onnx="$onnx_file"
    --saveEngine="$output"
    --minShapes="data:${BATCH_SIZE}x3x${INPUT_H}x${INPUT_W}"
    --optShapes="data:${BATCH_SIZE}x3x${INPUT_H}x${INPUT_W}"
    --maxShapes="data:${BATCH_SIZE}x3x${INPUT_H}x${INPUT_W}"
    --workspace="$WORKSPACE_MB"
  )

  case "$precision" in
    --fp16) args+=(--fp16) ;;
    --int8)
      args+=(--int8)
      if [ -n "$calib_dir" ]; then
        args+=(--calib="$calib_dir")
      fi
      ;;
  esac

  "$trtexec" "${args[@]}"
  echo ""
  echo "Engine saved to: $output"
}

cmd_export() {
  local pt_file="$1"; shift
  local imgsz_h=$INPUT_H
  local imgsz_w=$INPUT_W

  while [ $# -gt 0 ]; do
    case "$1" in
      --imgsz) imgsz_w="$2"; imgsz_h="$3"; shift 3 ;;
      *) echo "Unknown option: $1"; usage ;;
    esac
  done

  if [ ! -f "$pt_file" ]; then
    echo "Error: PyTorch model not found: $pt_file"
    exit 1
  fi

  local onnx_file="${pt_file%.pt}.onnx"

  echo "Exporting PyTorch model to ONNX:"
  echo "  Input:  $pt_file"
  echo "  Output: $onnx_file"
  echo "  Size:   ${imgsz_w}x${imgsz_h}"
  echo ""

  python3 -c "
from ultralytics import YOLO
model = YOLO('$pt_file')
model.export(format='onnx', imgsz=($imgsz_h, $imgsz_w), simplify=True, opset=17)
print('ONNX export complete: $onnx_file')
"
  echo "ONNX saved to: $onnx_file"
}

cmd_info() {
  local engine_file="$1"

  if [ ! -f "$engine_file" ]; then
    echo "Error: Engine file not found: $engine_file"
    exit 1
  fi

  local trtexec
  trtexec="$(find_trtexec)"
  if [ -z "$trtexec" ]; then
    echo "Error: trtexec not found."
    exit 1
  fi

  echo "Engine info: $engine_file"
  echo ""
  "$trtexec" --loadEngine="$engine_file" --dumpLayerInfo --exportLayerInfo=stdout 2>&1 | head -50
}

# --- Main ---
if [ $# -lt 2 ]; then
  usage
fi

case "$1" in
  onnx)   shift; cmd_onnx "$@" ;;
  export) shift; cmd_export "$@" ;;
  info)   shift; cmd_info "$@" ;;
  *)      usage ;;
esac
