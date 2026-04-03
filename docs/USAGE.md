# Usage Guide

## Prerequisites

### Hardware deployment (Jetson)
  NVIDIA Jetson (Orin or Xavier) with JetPack 5.x+
  ROS 2 Humble
  MAVROS + ArduPilot flight controller
  TensorRT 8.x+ (for detector)
  CUDA 11.x+ (for detector)

### Simulation (Gazebo SITL)
  Ubuntu 22.04
  ROS 2 Humble
  Gazebo Harmonic (gz-sim)
  ArduPilot SITL (sim_vehicle.py)
  ardupilot_gazebo plugin
  MAVROS

## Build

### Docker (recommended for development)

```bash
./scripts/build.sh              # build + test
./scripts/build.sh shell        # interactive shell
./scripts/build.sh build-image  # rebuild base image
```

### Native (on Jetson)

```bash
# Install dependencies
./scripts/deps/common.sh
./scripts/deps/jetson.sh

# Build
source /opt/ros/humble/setup.bash
colcon build --packages-select drone --cmake-args -DBUILD_TESTING=ON

# Test
colcon test --packages-select drone
colcon test-result --all --verbose
```

## Simulation (Gazebo SITL)

### Setup

Install ArduPilot SITL and Gazebo plugin:

```bash
# ArduPilot SITL
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
git submodule update --init --recursive

# Gazebo plugin
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && mkdir build && cd build
cmake .. && make -j$(nproc)
```

### Launch simulation

Terminal 1: Start Gazebo world

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -v4 -r iris_runway.sdf
```

Terminal 2: Start ArduPilot SITL

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

Terminal 3: Start MAVROS

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

Terminal 4: Start drone mission node

```bash
source install/setup.bash
ros2 launch drone drone.launch.py
```

### Quick simulation script

```bash
./scripts/sim.sh  # Starts all 4 terminals (requires gnome-terminal)
```

## Run

### Launch with defaults

```bash
source install/setup.bash
ros2 launch drone drone.launch.py
```

### Launch with custom parameters

```bash
ros2 launch drone drone.launch.py \
  mavros_ns:=/mavros/ \
  config_dir:=/path/to/custom/config
```

### Run node directly

```bash
ros2 run drone drone_node
```

## TensorRT Engine Building

Build TensorRT engines from trained YOLO models:

### Export PyTorch to ONNX

```bash
./scripts/build_engine.sh export model/best_circle.pt
./scripts/build_engine.sh export model/best_H.pt
```

### Build ONNX to TensorRT engine

```bash
# FP32 (default, highest accuracy)
./scripts/build_engine.sh onnx model/best_circle.onnx

# FP16 (recommended for Jetson, 2x faster)
./scripts/build_engine.sh onnx model/best_circle.onnx --fp16

# INT8 (fastest, requires calibration images)
./scripts/build_engine.sh onnx model/best_circle.onnx --int8 coco_calib/
```

### Inspect engine

```bash
./scripts/build_engine.sh info model/best_circle.engine
```

### Model files

Place model files in the `model/` directory:

| File | Purpose |
|------|---------|
| `model/best_circle.pt` | YOLOv8 circle detector (PyTorch) |
| `model/best_H.pt` | YOLOv8 H-marker detector (PyTorch) |
| `model/best_circle.onnx` | Circle detector (ONNX export) |
| `model/best_H.onnx` | H-marker detector (ONNX export) |
| `model/best_circle.engine` | Circle detector (TensorRT, GPU-specific) |
| `model/best_H.engine` | H-marker detector (TensorRT, GPU-specific) |

Note: `.engine` files are GPU-architecture-specific and must be rebuilt
for each target device (e.g., Jetson Orin vs Jetson Xavier).

## Configuration

All config files are in `config/`:

### mission.yaml

```yaml
headingangle_compass: 349.0  # Field orientation (degrees)
dx_shot: 0.0                 # Airdrop zone X offset from home (meters)
dy_shot: 30.0                # Airdrop zone Y offset from home (meters)
shot_halt: 4.5               # Cruise altitude (meters)
```

### pos_control.yaml

PID gains for position and velocity controllers. 10 PID parameter sets
(pos_x/y/z/yaw, pos_px/py/pz, pos_vx/vy/vz) plus velocity/acceleration limits
and fuzzy PID rules.

### airdrop.yaml

Airdrop visual-servo PID, approach limits, target pixel coordinates for
left/right servos, and drop point offsets.

### landing.yaml

Landing visual-servo PID, scout position, and surround search parameters.

### camera.yaml

Camera intrinsics (fx, fy, cx, cy), distortion coefficients, Kalman filter
parameters, device path, and TensorRT model paths.

## ROS 2 Topics

### Published

| Topic | Type | Hz | Description |
|-------|------|-----|-------------|
| `drone/state` | `std_msgs/Int32` | 20 | Current mission state encoding |

### Subscribed (via MAVROS)

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | Position, velocity, orientation |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS coordinates |
| `/mavros/imu/data` | `sensor_msgs/Imu` | IMU data |
| `/mavros/rangefinder/rangefinder` | `sensor_msgs/Range` | Altitude above ground |
| `/mavros/state` | `mavros_msgs/State` | Armed, connected, mode |
| `/mavros/altitude` | `mavros_msgs/Altitude` | Barometric altitude |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | Home position |

### Published (via MAVROS)

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Position commands |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Raw setpoints |
| `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | Global setpoints |
| `/mavros/setpoint_accel/accel` | `geometry_msgs/Vector3Stamped` | Acceleration commands |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Attitude commands |

## Mission Flow

1. **init**: Wait for GPS fix, capture start position, set home, initialize gimbal
2. **takeoff**: Arm and takeoff to 2m altitude
3. **goto_shot**: Fly to airdrop zone origin (12s timeout)
4. **airdrop**: Detect targets, approach, and release payload (up to 2 drops)
5. **goto_recon**: Fly to reconnaissance zone (7.5s timeout)
6. **recon_patrol**: Patrol waypoints through recon zone
7. **landing**: RTL, then visual approach to H-marker with surround search
8. **finished**: Wait 3s, shutdown

## Testing

```bash
# Run all tests
colcon test --packages-select drone
colcon test-result --all --verbose

# Run specific test
colcon test --packages-select drone --ctest-args -R test_e2e_system
```

### Test Suites (251 total)

| Suite | Tests | Layer |
|-------|-------|-------|
| test_timer | 6 | utils |
| test_rotate | 5 | utils |
| test_fuzzy_pid | 2 | control |
| test_math_utils | 5 | math |
| test_pid | 6 | math |
| test_basic_pid | 5 | math |
| test_scurve | 3 | math |
| test_detection_filter | 5 | perception |
| test_clustering | 4 | perception |
| test_camera_model | 6 | perception |
| test_yaw_utils | 14 | control |
| test_velocity_controller | 6 | control |
| test_cascade_controller | 5 | control |
| test_trajectory_controller | 7 | control |
| test_limits | 5 | control |
| test_frame_transforms | 9 | mission |
| test_mission_config | 2 | mission |
| test_fly_state | 2 | mission |
| test_waypoint_nav | 3 | mission |
| test_target_tracker | 4 | mission |
| test_integration | 9 | E2E |
| test_e2e_system | 5 | E2E |
