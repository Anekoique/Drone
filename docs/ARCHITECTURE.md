# System Architecture

Single-process drone flight system running on NVIDIA Jetson.
Camera capture, TensorRT inference, position control, and mission
logic all execute in one ROS 2 node with zero network hops.

## Module Dependency Graph

```
mission/  ─────────────────────────────────────────────────┐
  drone_node.hpp       (top-level node, state dispatch)    │
  mission_types.hpp    (FlyState, MissionConfig, Subsystems)│
  takeoff_handler.hpp  (init + takeoff)                    │
  airdrop_handler.hpp  (airdrop with visual servo)         │
  recon_handler.hpp    (reconnaissance patrol)             │
  landing_handler.hpp  (RTL + visual landing)              │
  target_tracker.hpp   (detection -> clustering -> world)  │
  frame_transforms.hpp (7 coordinate rotation functions)   │
  waypoint_nav.hpp     (time-based waypoint progression)   │
                                                           │
         ┌─────────────────────────────────────────────────┘
         ▼
control/  ─────────────────────────────────────────────────┐
  pos_control.hpp          (facade: composes all below)    │
  velocity_controller.hpp  (single-loop PID: pos -> vel)   │
  cascade_controller.hpp   (cascade PID: pos -> vel -> acc)│
  trajectory_controller.hpp(S-curve, circle, generator)    │
  mavros_commander.hpp     (6 MAVROS setpoint publishers)  │
  limits.hpp               (velocity/acceleration limits)  │
  yaw_utils.hpp            (angle normalization)           │
  fuzzy_pid.hpp            (adaptive PID with fuzzy rules) │
  autotune.hpp             (Ziegler-Nichols auto-tuning)   │
                                                           │
         ┌─────────────────────────────────────────────────┘
         ▼
drivers/  ─────────────────────────────────────────────────┐
  node_base.hpp   (ROS 2 Node with params + mode client)   │
  motors.hpp      (arm, takeoff, land, mode switch)        │
  inertial_nav.hpp(odometry, GPS, IMU, rangefinder)        │
  servo.hpp       (payload release via MAV_CMD_DO_SET_SERVO)│
  gimbal.hpp      (camera mount control)                   │
                                                           │
         ┌─────────────────────────────────────────────────┘
         ▼
perception/  ──────────────────────────────────────────────┐
  camera_driver.hpp    (V4L2/CSI capture)                  │
  camera_model.hpp     (pixel <-> world transforms)        │
  detection_filter.hpp (2D Kalman filter per class)        │
  detection_types.hpp  (Detection, Target, TargetSample)   │
  clustering.hpp       (k-means, k=3 for 3 containers)    │
  detection_adapter.hpp(to Detection2DArray conversion)    │
  detector/                                                │
    detector.hpp       (TensorRT dual-engine inference)    │
    config.hpp         (constexpr model parameters)        │
    preprocess.cu      (CUDA warp-affine resize)           │
    postprocess.hpp    (NMS, bbox decode)                  │
    plugin/yolo_layer  (custom TensorRT YOLO plugin)       │
                                                           │
         ┌─────────────────────────────────────────────────┘
         ▼
math/  ────────────────────────────────────────────────────┐
  pid.hpp       (full PID with anti-windup, filtering)     │
  basic_pid.hpp (simple 3-axis PID)                        │
  scurve.hpp    (S-curve trajectory primitives)            │
  types.hpp     (Eigen helpers: sq, norm, rotate_xy)       │
  utils.hpp     (predicates, constrain_float, safe_sqrt)   │
                                                           │
         ┌─────────────────────────────────────────────────┘
         ▼
utils/  ───────────────────────────────────────────────────┐
  readyaml.hpp  (YAML config loading via ament_index)      │
  timer.hpp     (steady_clock wrapper)                     │
  keyboard.hpp  (non-blocking terminal input)              │
  rotate.hpp    (2D rotation with C++20 concept)           │
trajectory/                                                │
  trajectory_generator.hpp (S-curve waypoint planner)      │
└──────────────────────────────────────────────────────────┘
```

Dependencies flow strictly downward: mission -> control -> drivers -> perception -> math -> utils.

## CMake Targets

| Target | ROS? | Links | Purpose |
|--------|------|-------|---------|
| `drone_utils` | No | yaml-cpp, Eigen3 | Math, PID, trajectory, utilities |
| `drone_control` | No | drone_utils | Position controllers (no ROS dep) |
| `drone_perception` | Yes | drone_utils, OpenCV | Camera, detection, clustering |
| `drone_perception_cuda` | CUDA | drone_perception, CUDA, TensorRT | TensorRT detector (conditional) |
| `drone_drivers` | Yes | drone_control, rclcpp, mavros_msgs | MAVROS interface + PosControl |
| `drone_mission` | Yes | drone_drivers, drone_perception | State machine, handlers, DroneNode |
| `drone_node` | Yes | drone_mission | Executable entry point |

## Data Flow

```
Camera (V4L2/CSI)
  │
  ▼
TensorRT YOLO Inference (GPU)
  │  circle detections + H-marker detections
  ▼
TargetTracker
  ├─ Kalman filter (per class)
  ├─ Pixel-to-world transform (CameraModel)
  └─ k-means clustering (3 containers)
       │  clustered world-frame targets
       ▼
Mission Handlers
  ├─ AirdropHandler: world_approach -> pixel_approach -> servo fire
  └─ LandingHandler: RTL -> surround search -> visual approach -> descent
       │  velocity/position commands
       ▼
PosControl
  ├─ VelocityController (PID: pos -> vel)
  ├─ CascadeController (PID: pos -> vel -> accel)
  └─ TrajectoryController (S-curve waypoints)
       │  setpoint messages
       ▼
MavrosCommander -> MAVROS -> ArduPilot Flight Controller
```

## State Machine

```
init -> takeoff -> goto_shot -> airdrop -> goto_recon -> recon_patrol -> landing -> finished
                                                                    └-> land_to_start -> finished
```

Each state is handled by a focused handler class. Transitions are explicit return
values (FlyState enum). DroneNode dispatches via a switch statement at 20 Hz.

## Coordinate Frames

| Frame | Convention | Used By |
|-------|-----------|---------|
| World (ENU) | East-North-Up | InertialNav, PosControl |
| Compass | Rotated by heading_compass_rad | Zone configs, waypoints |
| Body (NED) | North-East-Down | MAVROS setpoint_raw |
| Camera (ESD) | East-South-Down | CameraModel, detector |
| Pixel | (0,0) top-left, y-down | Detection output |

Frame transforms in `mission/frame_transforms.hpp`:
  `compass_to_world`, `world_to_compass`, `world_to_start`,
  `world_to_local`, `local_to_world`, `zone_origin_to_world`, `rotate_xy`

## Configuration Files

| File | Purpose |
|------|---------|
| `config/mission.yaml` | Compass heading, zone offsets, altitudes, servo PWM |
| `config/pos_control.yaml` | PID gains (10 sets), limits, fuzzy rules |
| `config/airdrop.yaml` | Airdrop PID, approach limits, target pixels, drop points |
| `config/landing.yaml` | Landing PID, scout position, surround search params |
| `config/camera.yaml` | Camera intrinsics, distortion, model paths, Kalman params |
| `config/position.yaml` | Legacy position config (reference) |
