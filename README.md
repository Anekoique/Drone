# Drone

Autonomous multirotor UAV mission system for airdrop and reconnaissance competitions.

Refactored and integrated from repos: [drone](https://github.com/dtyjno/drone) | [cv_py](https://github.com/ak47k98/ros2_yolov8) | [cv_cpp](https://github.com/Anekoique/Ros2-Yolov5) By AGENTS(Claude Opus 4.6) which is FOR REFERENCE ONLY.
 
ROS 2 Humble / C++20 / NVIDIA Jetson / ArduPilot

## Quick Start

```bash
# Build (Docker)
./scripts/build.sh

# Simulation (Gazebo SITL)
./scripts/sim.sh

# Run on hardware
source install/setup.bash
ros2 launch drone drone.launch.py
```

## Architecture

Single C++ process on NVIDIA Jetson. Camera capture, TensorRT YOLO inference,
detection filtering, position control, and mission logic all execute in one
ROS 2 node with zero network hops.

```
Camera -> TensorRT YOLO -> Kalman Filter -> Clustering -> Mission Handlers
                                                              |
                                                     PosControl (PID cascade)
                                                              |
                                                     MAVROS -> ArduPilot
```

See [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md) for the full module dependency
graph, data flow, coordinate frames, and CMake target layout.

## Mission

Defined in [docs/PROBLEM.md](./docs/PROBLEM.md):

1. Take off autonomously
2. Fly to the drop zone, detect three containers, drop two payloads
3. Fly to the reconnaissance zone, identify ground markers
4. Return and land on the H-marker using visual servo

## Project Structure

```
include/drone/
  utils/        readyaml, timer, keyboard, rotate
  math/         pid, basic_pid, scurve, types, utils
  perception/   camera_model, detection_filter, clustering, detector (TensorRT)
  control/      pos_control, velocity/cascade/trajectory controllers, mavros_commander
  drivers/      node_base, motors, inertial_nav, servo, gimbal
  mission/      drone_node, handlers (takeoff, airdrop, recon, landing), target_tracker
src/            Matching .cpp implementations + main.cpp
test/           22 test suites, 251 tests
config/         6 YAML config files
launch/         ROS 2 launch file
scripts/        build.sh, sim.sh, build_engine.sh
model/          TensorRT engine files (not tracked, see scripts/build_engine.sh)
```

## Build and Test

```bash
./scripts/build.sh              # Docker build + test (251 tests)
./scripts/build.sh shell        # Interactive shell in container
colcon test --packages-select drone  # Run tests natively
```

## TensorRT Engine Building

```bash
./scripts/build_engine.sh export model/best_circle.pt         # PyTorch -> ONNX
./scripts/build_engine.sh onnx model/best_circle.onnx --fp16  # ONNX -> TensorRT
```

## Hardware

- NVIDIA Jetson Orin/Xavier (onboard compute)
- ArduPilot flight controller (via MAVROS)
- Camera (V4L2/CSI)
- Servo-actuated payload release mechanism

## Documentation

- [Architecture](./docs/ARCHITECTURE.md): module graph, data flow, coordinate frames
- [Usage Guide](./docs/USAGE.md): build, simulation, run, configuration, testing
- [Mission Specification](./docs/PROBLEM.md): competition rules
- [Dependency Analysis](./docs/DEPENDENCIES.md): MAVROS topic graph
- [Refactoring Roadmap](./docs/ROADMAP.md): 8-phase migration plan
- [Progress](./docs/PROGRESS.md): phase-by-phase completion status

## License

MPL-2.0. Copyright (c) 2024-2026 HDU-DXY-Team.
