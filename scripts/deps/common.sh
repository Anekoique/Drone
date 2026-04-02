#!/usr/bin/env bash
# Shared helpers and dependency logic for local.sh and raspi.sh.
# Not meant to be run directly — sourced by the target scripts.
set -euo pipefail

readonly ROS_DISTRO="humble"
readonly MAVROS_GEO_SCRIPT="https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh"

log()  { printf "\n\033[1;32m>>> %s\033[0m\n" "$*"; }
warn() { printf "\033[1;33mWARN: %s\033[0m\n" "$*"; }

preflight() {
  if [[ $EUID -ne 0 ]]; then
    echo "Run as root: sudo $0" >&2; exit 1
  fi
  local codename
  codename="$(lsb_release -sc 2>/dev/null || true)"
  if [[ "$codename" != "jammy" ]]; then
    echo "Error: requires Ubuntu 22.04 (jammy), got '$codename'." >&2
    [[ "${1:-}" == "--force" ]] || exit 1
    warn "Forcing install on non-Jammy host."
  fi
}

setup_ros_repo() {
  log "Setting up ROS 2 apt repository"
  apt-get update -qq
  apt-get install -y -qq curl gnupg2 lsb-release software-properties-common
  add-apt-repository -y universe

  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update -qq
}

# Packages required by CMakeLists.txt on both targets.
install_core() {
  log "Installing MAVROS and build dependencies"
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-visualization-msgs \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    build-essential \
    cmake \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libncurses-dev
}

# MAVROS refuses to start without the egm96-5 geoid dataset.
install_geographiclib() {
  log "Installing GeographicLib datasets"
  apt-get install -y -qq geographiclib-tools 2>/dev/null || true

  if command -v geographiclib-get-geoids &>/dev/null; then
    geographiclib-get-geoids   egm96-5  2>/dev/null || true
    geographiclib-get-gravity  egm96    2>/dev/null || true
    geographiclib-get-magnetic emm2015  2>/dev/null || true
  else
    local tmp; tmp="$(mktemp)"
    curl -sSL "$MAVROS_GEO_SCRIPT" -o "$tmp"
    bash "$tmp" || warn "GeographicLib dataset install failed"
    rm -f "$tmp"
  fi
}

setup_rosdep() {
  log "Initializing rosdep"
  [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init 2>/dev/null || true
  sudo -u "${SUDO_USER:-$USER}" rosdep update --rosdistro="${ROS_DISTRO}" 2>/dev/null || true
}
