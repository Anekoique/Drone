# Drone

Autonomous multirotor UAV mission system for airdrop and reconnaissance.

## Architecture

Single C++ process on NVIDIA Jetson. Camera capture → TensorRT YOLO inference
→ detection filtering → mission control → MAVROS → flight controller.
Zero network hops.

## Mission

Defined in [docs/PROBLEM.md](./docs/PROBLEM.md):

- take off autonomously,
- fly to the drop zone,
- detect three containers and drop two payloads,
- fly to the reconnaissance zone,
- identify ground markers,
- return and land safely on the takeoff/landing point.

## Core Functions

- Camera capture and TensorRT YOLO detection (on Jetson GPU)
- Kalman-filtered target tracking and pixel↔world coordinate transforms
- Autonomous takeoff, navigation, and landing
- Payload drop planning and release
- Visual guidance for drop and landing tasks

## Hardware

- NVIDIA Jetson (onboard compute, replaces Raspberry Pi)
- ArduPilot flight controller (via MAVROS)
- Camera (V4L2/CSI)
- Servo-actuated payload release mechanism

## References

- Mission specification: [docs/PROBLEM.md](./docs/PROBLEM.md)
- Dependency analysis: [docs/DEPENDENCIES.md](./docs/DEPENDENCIES.md)
- Refactoring roadmap: [docs/ROADMAP.md](./docs/ROADMAP.md)
