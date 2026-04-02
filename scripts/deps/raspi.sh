#!/usr/bin/env bash
# Install dependencies for the onboard Raspberry Pi.
# Target: Ubuntu 22.04 (Jammy) arm64
# Usage:  sudo ./scripts/deps/raspi.sh [--force]
source "$(dirname "$0")/common.sh"

preflight "${1:-}"
setup_ros_repo

log "Installing ROS 2 ${ROS_DISTRO} base"
apt-get install -y -qq ros-${ROS_DISTRO}-ros-base

install_core
install_geographiclib
setup_rosdep

log "Done. Next: source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"
