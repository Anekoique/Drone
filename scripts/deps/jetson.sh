#!/usr/bin/env bash
# Install dependencies for the onboard NVIDIA Jetson.
# Target: JetPack 6 / Ubuntu 22.04 (Jammy) arm64
# Assumes JetPack is already installed (provides CUDA, cuDNN, TensorRT).
# Usage:  sudo ./scripts/deps/jetson.sh [--force]
source "$(dirname "$0")/common.sh"

preflight "${1:-}"
setup_ros_repo

log "Installing ROS 2 ${ROS_DISTRO} base"
apt-get install -y -qq ros-${ROS_DISTRO}-ros-base

install_core

# OpenCV with CUDA support (JetPack provides this, but ensure dev headers)
log "Installing perception dependencies"
apt-get install -y -qq \
  libopencv-dev \
  v4l-utils

# Verify JetPack components are present
log "Checking JetPack components"
if ! dpkg -l | grep -q libnvinfer-dev; then
  warn "TensorRT dev package not found. Ensure JetPack is installed."
fi
if ! command -v nvcc &>/dev/null; then
  warn "CUDA toolkit (nvcc) not found. Ensure JetPack is installed."
fi

install_geographiclib
setup_rosdep

log "Done. Next: source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"
