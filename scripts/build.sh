#!/usr/bin/env bash
# Build and test the drone package in a ROS 2 Humble Docker container.
# The base image (deps) is built once and cached. Source code is mounted
# at runtime — no rebuild needed when code changes.
#
# Usage:
#   ./scripts/build.sh              # build + test
#   ./scripts/build.sh shell        # interactive shell
#   ./scripts/build.sh build-image  # force rebuild base image
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_NAME="drone-dev"

RUN_CMD=(
  docker run --rm
  -v "$PROJECT_DIR":/Drone/src/drone
  -w /Drone
  "$IMAGE_NAME"
)

ensure_image() {
  if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "Building base image (first time only)..."
    docker build -t "$IMAGE_NAME" "$PROJECT_DIR"
  fi
}

case "${1:-}" in
  build-image)
    echo "Rebuilding base image..."
    docker build -t "$IMAGE_NAME" "$PROJECT_DIR"
    ;;
  shell)
    ensure_image
    docker run -it --rm \
      -v "$PROJECT_DIR":/Drone/src/drone \
      -w /Drone \
      "$IMAGE_NAME" \
      bash
    ;;
  *)
    ensure_image
    echo "Building..."
    "${RUN_CMD[@]}" bash -c \
      "source /opt/ros/humble/setup.bash && colcon build --packages-select drone"
    echo "Testing..."
    "${RUN_CMD[@]}" bash -c \
      "source /opt/ros/humble/setup.bash && colcon build --packages-select drone && colcon test --packages-select drone && colcon test-result --all --verbose"
    echo "All passed."
    ;;
esac
