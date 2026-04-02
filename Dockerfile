# Base image with all ROS 2 deps pre-installed.
# Source code is NOT baked in — mount it at runtime.
FROM ros:humble-ros-base

RUN apt-get update -qq && apt-get install -y -qq \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-humble-ament-cmake \
      ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common \
      ros-humble-eigen3-cmake-module \
      ros-humble-ament-index-cpp \
      libeigen3-dev \
      libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /Drone
SHELL ["/bin/bash", "-c"]

# Entrypoint sources ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
