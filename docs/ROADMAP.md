# Refactoring Roadmap

Migrate the legacy `px4_ros_com` monolith into a clean, modular `Drone` project.

## Guiding Principles

- Move bottom-up: leaf modules first, god class last.
- Each phase produces a compilable project — no big-bang rewrites.
- Split headers from implementations; no more logic-in-headers.
- Break circular deps by introducing interfaces.
- Keep the ROS 2 / MAVROS integration intact at all times.

---

## Phase 0 — Project Skeleton

Set up the `Drone/` project structure, build system, documentation, and
dependency install scripts. No C++ code yet.

Deliverables:
- [x] `README.md`, `CLAUDE.md`, `AGENTS.md`
- [x] `docs/PROBLEM.md` (competition rules)
- [x] `docs/DEPENDENCIES.md` (dependency analysis)
- [x] `scripts/deps/` (common.sh, local.sh, raspi.sh)
- [ ] `CMakeLists.txt` (new, targeting `drone` package name)
- [ ] `package.xml` (new, clean dependency list)
- [ ] `config/` (migrate YAML configs)
- [ ] `launch/` (migrate launch files)
- [ ] `.clang-format` (enforce consistent style)

---

## Phase 1 — Pure Utilities (leaves, zero project deps)

Extract standalone modules that have no project-local `#include` chains.
These can be moved and tested in isolation.

| Module | Legacy File | Notes |
|---|---|---|
| `math_utils` | `math_utils.h` | `is_zero`, `is_positive`, etc. |
| `readyaml` | `Readyaml.h` | YAML config reader wrapper |
| `timer` | `utils.h` (Timer class) | `Timer`, `rotate_angle` |
| `keyboard` | `utils.h` (_kbhit, _getch) | Terminal input helpers |
| `fuzzy_pid` | `FuzzyPID.h/cpp` | Fuzzy PID logic |
| `autotune` | `AutoTune.h/cpp` | PID auto-tuning |
| `trajectory_generator` | `TrajectoryGenerator.h/cpp` | S-curve trajectory |

Tasks:
- [ ] Create `src/utils/` directory for pure utilities
- [ ] Move each module, split .h/.cpp properly
- [ ] Remove dead code (`AC_WPNav.h/cpp` — all commented out)
- [ ] Remove backup files (`Yolo.h.bak`, `Yolo copy.bak`)
- [ ] Write unit tests for math_utils, Timer, trajectory_generator

---

## Phase 2 — Math & Sensor Types

Extract the Eigen typedef layer and sensor data types.

| Module | Legacy File | Deps |
|---|---|---|
| `math` | `math.h/cpp` | math_utils |
| `pid` | `PID.h/cpp` | readyaml |
| `mypid` | `MYPID.h` | readyaml |
| `scurve` | `SCurve.h/cpp` | math |

Tasks:
- [ ] Create `src/math/` for math.h, math_utils.h, PID, SCurve
- [ ] Replace `#define` PID gains with constexpr or config-loaded values
- [ ] Remove `using namespace Eigen` from headers — use `Eigen::` prefix
- [ ] Remove `#undef MIN` / `#undef MAX` hacks, use `std::min`/`std::max`
- [ ] Write unit tests for PID controller

---

## Phase 3 — Perception

Extract detection and camera modules. These depend only on Phase 1–2 modules.

| Module | Legacy File | Deps |
|---|---|---|
| `kalman_filter_2d` | `Yolo.h` (KalmanFilter2D class) | (Eigen only) |
| `detection_subscriber` | `Yolo.h` (YOLO class) | readyaml, kalman_filter_2d |
| `camera_model` | `CameraGimbal.h` (Camera class) | math, readyaml, OpenCV |
| `gimbal_controller` | `CameraGimbal.h` (CameraGimbal class) | camera_model |
| `clustering` | `clustering.h/cpp` | math |

Tasks:
- [ ] Create `src/perception/` directory
- [ ] Extract `KalmanFilter2D` into its own header
- [ ] Split `Camera` (pure geometry) from `CameraGimbal` (ROS publisher)
- [ ] Break `clustering.h` → `OffboardControl.h` circular dep by passing
      `Vector3d` points instead of depending on the god class
- [ ] Write unit tests for Camera pixel↔world transforms, KalmanFilter2D

---

## Phase 4 — Infrastructure (ROS node base, MAVROS clients)

Extract the ROS 2 node infrastructure and MAVROS communication layer.

| Module | Legacy File | Deps |
|---|---|---|
| `node_base` | `OffboardControl_Base.h/cpp` | math |
| `motors` | `Motors.h/cpp` | node_base |
| `inertial_nav` | `InertialNav.h/cpp` | node_base, math |
| `servo_controller` | `ServoController.h` | node_base |

Tasks:
- [ ] Create `src/drivers/` for hardware-interface modules
- [ ] Make `InertialNav` fields private, expose const getters
- [ ] Replace raw `OffboardControl_Base*` back-pointers with
      `rclcpp::Node::SharedPtr` or interface references
- [ ] Extract QoS profile setup into a shared helper
- [ ] Write integration tests against mock MAVROS topics

---

## Phase 5 — Position Control

Extract the position controller — the most complex intermediate module.

| Module | Legacy File | Deps |
|---|---|---|
| `pos_control` | `PosControl.h/cpp` | node_base, inertial_nav, pid, fuzzy_pid, trajectory_generator, autotune |

Tasks:
- [ ] Create `src/control/`
- [ ] Move `#define` PID constants → config file or constexpr
- [ ] Split PosControl into:
  - `VelocityController` (PID-based velocity commands)
  - `PositionController` (position→velocity cascade)
  - `TrajectoryFollower` (S-curve / waypoint following)
- [ ] Extract fuzzy PID rule tables to YAML config
- [ ] Write unit tests for position→velocity cascade

---

## Phase 6 — State Machine & Mission Logic

Refactor the state machine and break apart the god class.

| Module | Legacy File | Deps |
|---|---|---|
| `state_machine` | `StateMachine.h/cpp` | (forward decl only) |
| `mission_controller` | `OffboardControl.h/cpp` | everything |

Tasks:
- [ ] Create `src/mission/`
- [ ] Extract mission-specific logic from OffboardControl into:
  - `TakeoffHandler`
  - `AirdropHandler` (Doshot, surrounding_shot)
  - `ReconHandler` (Surround_see)
  - `LandingHandler` (Doland, LandToStart)
- [ ] Make `StateMachine` generic — remove `friend class` relationship
- [ ] Replace coordinate rotation methods scattered in OffboardControl
      with a single `FrameTransforms` utility
- [ ] Move waypoint definitions to config files
- [ ] Replace `OffboardControl` god class with a thin `MissionNode` that
      composes the above handlers via dependency injection

---

## Phase 7 — Config & Launch

Finalize configuration and launch infrastructure.

Tasks:
- [ ] Consolidate YAML configs (OffboardControl.yaml, pos_config.yaml,
      can_config.yaml, land_config.yaml, camera.yaml) into a structured
      config directory with clear naming
- [ ] Create ROS 2 launch files (Python or YAML) with parameter loading
- [ ] Add sim_mode / debug_mode as launch arguments
- [ ] Document all config parameters

---

## Phase 8 — Testing & CI

Tasks:
- [ ] Set up GoogleTest in CMakeLists.txt
- [ ] Achieve 80%+ coverage on math, PID, camera, and clustering modules
- [ ] Add integration tests with mock MAVROS
- [ ] Set up GitHub Actions CI (build + test on Ubuntu 22.04)
- [ ] Add clang-format check to CI
- [ ] Add clang-tidy check to CI

---

## Module Dependency Graph (target architecture)

```
src/
  utils/          math_utils, timer, keyboard, readyaml
  math/           math, pid, mypid, scurve, fuzzy_pid, autotune
  perception/     kalman_filter_2d, detection_subscriber, camera_model,
                  gimbal_controller, clustering
  drivers/        node_base, motors, inertial_nav, servo_controller
  control/        velocity_controller, position_controller, trajectory_follower
  mission/        state_machine, takeoff, airdrop, recon, landing, mission_node
  main.cpp
```

Dependencies flow strictly downward: `mission → control → drivers → perception → math → utils`.
No upward or circular dependencies.
