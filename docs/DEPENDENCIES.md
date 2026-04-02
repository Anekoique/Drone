# Dependencies

## Overview

| Target | OS | Hardware | ROS 2 |
|---|---|---|---|
| NVIDIA Jetson (onboard) | JetPack 6 / Ubuntu 22.04 arm64 | ARM CPU + NVIDIA GPU | Humble |
| Local workstation (dev) | Ubuntu 22.04 x86_64 | x86 CPU + NVIDIA GPU | Humble |

The Jetson replaces the Raspberry Pi as the onboard computer. It runs the
complete pipeline in a single process: camera capture → TensorRT inference →
control → MAVROS. No network hops, no split deployment.

The local workstation is used for development, SITL simulation, and debugging.

---

## Shared Core

### MAVROS

| Package | Purpose |
|---|---|
| `ros-humble-mavros` | MAVLink-to-ROS 2 bridge |
| `ros-humble-mavros-extras` | Mount control, rangefinder, etc. |

### ROS 2 Message Packages

| Package | Used For |
|---|---|
| `geometry_msgs` | PoseStamped, TwistStamped, Vector3Stamped |
| `nav_msgs` | Odometry |
| `sensor_msgs` | NavSatFix, Imu, Range, Image |
| `vision_msgs` | Detection2DArray (internal detector output) |
| `visualization_msgs` | MarkerArray (rviz targets) |
| `geographic_msgs` | GeoPoseStamped (global setpoints) |
| `trajectory_msgs` | MultiDOFJointTrajectory |
| `mavros_msgs` | State, MountControl, CommandBool, SetMode, etc. |
| `tf2`, `tf2_ros` | Coordinate frame transforms |
| `cv_bridge` | ROS Image <-> OpenCV Mat |

### C++ Libraries

| Library | Used For |
|---|---|
| Eigen3 | Vector/matrix math, quaternions |
| OpenCV | Camera capture, distortion correction, image processing |
| yaml-cpp | YAML config parsing |
| TensorRT | YOLO inference (GPU accelerated) |
| CUDA | GPU compute for TensorRT and preprocessing |

### Build Tools

| Tool | Package |
|---|---|
| CMake >= 3.14 | `cmake` |
| colcon | `python3-colcon-common-extensions` |
| rosdep | `python3-rosdep` |
| eigen3_cmake_module | `ros-humble-eigen3-cmake-module` |
| CUDA Toolkit | JetPack (Jetson) or `nvidia-cuda-toolkit` (workstation) |

### GeographicLib

MAVROS requires GeographicLib datasets:

- `egm96-5` (geoid) — **mandatory**
- `egm96` (gravity)
- `emm2015` (magnetic)

---

## Jetson Only

| Component | Source |
|---|---|
| JetPack 6 | NVIDIA SDK (includes CUDA, cuDNN, TensorRT) |
| V4L2 / CSI camera driver | Built into JetPack kernel |
| TensorRT | JetPack (pre-installed) |

The Jetson runs the single `drone_node` binary which handles everything:
camera → inference → control → MAVROS.

---

## Local Workstation Only

| Component | Source |
|---|---|
| Gazebo | `ros-humble-ros-gz` or standalone install |
| ArduPilot SITL | Built from source (`sim_vehicle.py`) |
| TensorRT | NVIDIA TensorRT (for local inference testing) |
| `pal-statistics-msgs` | Optional compile-time statistics publisher |

---

## Runtime Data Flow

Single process on Jetson, zero network hops:

```
Camera (V4L2/CSI)
  │
  ▼
TensorRT YOLO inference (GPU)
  │
  ▼
Detection filter (Kalman) + Clustering
  │
  ▼
Camera model (pixel → world)
  │
  ▼
Mission state machine
  ├── Airdrop handler → Servo (MAVROS cmd/command)
  ├── Recon handler
  └── Landing handler
  │
  ▼
Position controller (PID cascade)
  │
  ▼
MAVROS → Flight Controller
```

### MAVROS Topics Consumed

| Topic | Type | Consumer |
|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | InertialNav |
| `/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | InertialNav |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | InertialNav |
| `/mavros/rangefinder/rangefinder` | `sensor_msgs/msg/Range` | InertialNav |
| `/mavros/altitude` | `mavros_msgs/msg/Altitude` | InertialNav |
| `/mavros/state` | `mavros_msgs/msg/State` | Motors |
| `/mavros/home_position/home` | `mavros_msgs/msg/HomePosition` | Motors |
| `/mavros/mount_control/status` | `geometry_msgs/msg/Vector3Stamped` | Gimbal |

### MAVROS Topics Published

| Topic | Type | Publisher |
|---|---|---|
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/msg/TwistStamped` | PosControl |
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | PosControl |
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | PosControl |
| `/mavros/setpoint_raw/global` | `mavros_msgs/msg/GlobalPositionTarget` | PosControl |
| `/mavros/setpoint_accel/accel` | `geometry_msgs/msg/Vector3Stamped` | PosControl |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | PosControl |
| `/mavros/mount_control/command` | `mavros_msgs/msg/MountControl` | Gimbal |

### MAVROS Services Called

| Service | Type | Caller |
|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | Motors |
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Motors |
| `/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | Motors |
| `/mavros/cmd/land` | `mavros_msgs/srv/CommandTOL` | Motors |
| `/mavros/cmd/set_home` | `mavros_msgs/srv/CommandHome` | Motors |
| `/mavros/cmd/command` | `mavros_msgs/srv/CommandLong` | Servo |
| `/mavros/param/set` | `mavros_msgs/srv/ParamSetV2` | Motors |

---

## Setup Scripts

```
scripts/deps/
  common.sh   — shared logic (sourced, not run directly)
  local.sh    — workstation: desktop + Gazebo + CUDA/TensorRT
  jetson.sh   — Jetson: ros-base + JetPack deps
```

After cloning the workspace, resolve transitive deps:
```sh
rosdep install --from-paths src --ignore-src -r -y
```
