#!/usr/bin/env bash
# Install dependencies for the local development workstation.
# Target: Ubuntu 22.04 (Jammy) x86_64
# Usage:  sudo ./scripts/deps/local.sh [--force]
source "$(dirname "$0")/common.sh"

preflight "${1:-}"
setup_ros_repo

log "Installing ROS 2 ${ROS_DISTRO} desktop"
apt-get install -y -qq ros-${ROS_DISTRO}-desktop

install_core

# Simulation (Gazebo)
log "Installing simulation dependencies"
apt-get install -y -qq ros-${ROS_DISTRO}-ros-gz \
  || warn "ros-gz not available; install Gazebo manually"

# Optional: pal_statistics (compile-time feature flag)
apt-get install -y -qq ros-${ROS_DISTRO}-pal-statistics-msgs 2>/dev/null \
  || warn "pal-statistics-msgs not found; stats publisher disabled at build time"

install_geographiclib
setup_rosdep

log "Done. Next: source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"
