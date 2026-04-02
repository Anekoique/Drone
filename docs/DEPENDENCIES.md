# Dependencies

## Overview

| Target | OS | ROS 2 Meta-package |
|---|---|---|
| Local workstation | Ubuntu 22.04 (Jammy) x86_64 | `ros-humble-desktop` |
| Raspberry Pi | Ubuntu 22.04 (Jammy) arm64 | `ros-humble-ros-base` |

Both targets share a common core installed by `scripts/deps/common.sh`.

---

## Shared Core

### MAVROS

| Package | Purpose |
|---|---|
| `ros-humble-mavros` | MAVLink-to-ROS 2 bridge |
| `ros-humble-mavros-extras` | Mount control, rangefinder, etc. |

### ROS 2 Message Packages

Required by `CMakeLists.txt`:

| Package | Used For |
|---|---|
| `geometry_msgs` | PoseStamped, TwistStamped, Vector3Stamped |
| `nav_msgs` | Odometry |
| `sensor_msgs` | NavSatFix, Imu, Range, Image |
| `vision_msgs` | Detection2DArray (external detector) |
| `visualization_msgs` | MarkerArray (rviz targets) |
| `geographic_msgs` | GeoPoseStamped (global setpoints) |
| `trajectory_msgs` | MultiDOFJointTrajectory |
| `mavros_msgs` | State, MountControl, CommandBool, SetMode, etc. |
| `tf2`, `tf2_ros` | Coordinate frame transforms |
| `cv_bridge` | ROS Image <-> OpenCV Mat |

### C++ Libraries

| Library | apt Package | Used For |
|---|---|---|
| Eigen3 | `libeigen3-dev` | Vector/matrix math, quaternions |
| OpenCV | `libopencv-dev` | Camera distortion correction |
| yaml-cpp | `libyaml-cpp-dev` | YAML config parsing |
| ncurses | `libncurses-dev` | Terminal keyboard input (debug mode) |

### Build Tools

| Tool | apt Package |
|---|---|
| CMake >= 3.5 | `cmake` |
| colcon | `python3-colcon-common-extensions` |
| rosdep | `python3-rosdep` |
| eigen3_cmake_module | `ros-humble-eigen3-cmake-module` |

### GeographicLib

MAVROS refuses to start without GeographicLib datasets. The scripts install
`geographiclib-tools` then download:

- `egm96-5` (geoid) — **mandatory**
- `egm96` (gravity)
- `emm2015` (magnetic)

---

## Local Workstation Only

| Component | Source |
|---|---|
| Gazebo | `ros-humble-ros-gz` or standalone install |
| ArduPilot SITL | Built from source (`sim_vehicle.py`) |
| `pal-statistics-msgs` | Optional compile-time statistics publisher |

---

## Raspberry Pi Only

Uses `ros-humble-ros-base` instead of `desktop` (~2 GB smaller). The shared
core packages are installed explicitly. Does not need Gazebo, rviz, SITL, or
pal_statistics.

---

## External Runtime Dependencies

### External detector (not included)

| Topic | Type |
|---|---|
| `detection2d_array` | `vision_msgs/msg/Detection2DArray` |

Expected detection classes: `circle`, `h`, `stuffed`.

### Topics consumed from MAVROS

| Topic | Type | Consumer |
|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | InertialNav |
| `/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | InertialNav |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | InertialNav |
| `/mavros/rangefinder/rangefinder` | `sensor_msgs/msg/Range` | InertialNav |
| `/mavros/altitude` | `mavros_msgs/msg/Altitude` | InertialNav |
| `/mavros/state` | `mavros_msgs/msg/State` | Motors |
| `/mavros/home_position/home` | `mavros_msgs/msg/HomePosition` | Motors |
| `/mavros/mount_control/status` | `geometry_msgs/msg/Vector3Stamped` | CameraGimbal |

### Topics published into MAVROS

| Topic | Type | Publisher |
|---|---|---|
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/msg/TwistStamped` | PosControl |
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | PosControl |
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | PosControl |
| `/mavros/setpoint_raw/global` | `mavros_msgs/msg/GlobalPositionTarget` | PosControl |
| `/mavros/setpoint_accel/accel` | `geometry_msgs/msg/Vector3Stamped` | PosControl |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | PosControl |
| `/mavros/mount_control/command` | `mavros_msgs/msg/MountControl` | CameraGimbal |

### MAVROS services called

| Service | Type | Caller |
|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | Motors |
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Motors |
| `/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | Motors |
| `/mavros/cmd/land` | `mavros_msgs/srv/CommandTOL` | Motors |
| `/mavros/cmd/set_home` | `mavros_msgs/srv/CommandHome` | Motors |
| `/mavros/cmd/command` | `mavros_msgs/srv/CommandLong` | ServoController |
| `/mavros/param/set` | `mavros_msgs/srv/ParamSetV2` | Motors |

---

## Setup Scripts

```
scripts/deps/
  common.sh   — shared logic (sourced, not run directly)
  local.sh    — workstation: desktop + Gazebo + pal_statistics
  raspi.sh    — Pi: ros-base only
```

Both run: repo setup -> ROS 2 install -> core deps -> GeographicLib -> rosdep init.

After cloning the workspace, resolve transitive deps manually:
```sh
rosdep install --from-paths src --ignore-src -r -y
```
