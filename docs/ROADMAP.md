# Refactoring Roadmap

Migrate the legacy `px4_ros_com` control code and `cv_py`/`cv_cpp` vision code
into a single, modular `Drone` package running on NVIDIA Jetson.

## Target Architecture

Single process on Jetson. Camera → TensorRT inference → control → MAVROS.
Zero network hops.

```
include/drone/
  utils/              readyaml, timer, keyboard, rotate
  math/               math_utils, math, pid, scurve, fuzzy_pid, autotune
  perception/
    camera_driver.hpp     V4L2/CSI capture
    detector.hpp          TensorRT YOLO inference
    detection_filter.hpp  Kalman filter + tracking
    camera_model.hpp      pixel↔world transforms
    clustering.hpp        container center finding
  control/
    pos_control.hpp       PID cascade position controller
    trajectory.hpp        S-curve trajectory generation
  drivers/
    motors.hpp            MAVROS arm/mode/takeoff/land
    inertial_nav.hpp      odometry/GPS/IMU/rangefinder
    servo.hpp             payload release
    gimbal.hpp            camera gimbal control
  mission/
    state_machine.hpp
    takeoff.hpp
    airdrop.hpp           includes servo decision logic (from cv_py)
    recon.hpp
    landing.hpp
    drone_node.hpp        top-level node composing all above

src/
  (matching .cpp files)
  main.cpp              single entry point
```

Dependencies flow strictly downward:
```
mission → control → drivers → perception → math → utils
```

## Guiding Principles

- Move bottom-up: leaf modules first, god class last.
- Each phase produces a compilable project.
- Split headers (.hpp in include/) from implementations (.cpp in src/).
- No `using namespace` in headers.
- Single executable, single process on Jetson.

---

## Phase 0 — Project Skeleton `[DONE]`

Build system, config, formatting, dependency scripts.

Deliverables:
- [x] README, CLAUDE.md, AGENTS.md, docs/PROBLEM.md
- [x] docs/DEPENDENCIES.md
- [x] scripts/deps/ (common.sh, local.sh, jetson.sh)
- [x] CMakeLists.txt (C++20, ament_cmake)
- [x] package.xml (format 3, MPL-2.0)
- [x] config/ (5 YAML files migrated)
- [x] .clang-format (ament/autoware conventions)

---

## Phase 1 — Pure Utilities `[DONE]`

Leaf modules with zero project-local dependencies.

| Module | Legacy Source | Target |
|---|---|---|
| readyaml | `Readyaml.h` | `include/drone/utils/readyaml.hpp` |
| timer | `utils.h` (Timer) | `include/drone/utils/timer.hpp` |
| keyboard | `utils.h` (_kbhit/_getch) | `include/drone/utils/keyboard.hpp` |
| rotate | `utils.h` (rotate_angle) | `include/drone/utils/rotate.hpp` |
| fuzzy_pid | `FuzzyPID.h/cpp` | `include/drone/control/fuzzy_pid.hpp` |
| autotune | `AutoTune.h/cpp` | `include/drone/control/autotune.hpp` |
| trajectory_gen | `TrajectoryGenerator.h/cpp` | `include/drone/trajectory/trajectory_generator.hpp` |

---

## Phase 2 — Math & PID `[DONE]`

Modules depending on Phase 1 leaves.

| Module | Legacy Source | Target |
|---|---|---|
| utils | `math_utils.h` | `include/drone/math/utils.hpp` |
| types | `math.h/cpp` | `include/drone/math/types.hpp` |
| pid | `PID.h/cpp` | `include/drone/math/pid.hpp` |
| basic_pid | `MYPID.h` | `include/drone/math/basic_pid.hpp` |
| scurve | `SCurve.h/cpp` | `include/drone/math/scurve.hpp` |

Tasks:
- Replace `#define` PID gains with constexpr or config
- Remove `using namespace Eigen` from headers
- Replace MIN/MAX macros with `std::min`/`std::max`

---

## Phase 3 — Perception `[DONE]`

Camera, detection, and target processing. Merges legacy drone perception
code with cv_cpp TensorRT inference.

| Module | Source | Target |
|---|---|---|
| camera_driver | `cv_cpp/image_pub.cpp`, `cv_py/camera_capture.py` | `include/drone/perception/camera_driver.hpp` |
| detector | `cv_cpp/trtdet/` (TensorRT), `cv_py/detect.py` (logic) | `include/drone/perception/detector.hpp` |
| detection_filter | `Yolo.h` (KalmanFilter2D + subscriber) | `include/drone/perception/detection_filter.hpp` |
| camera_model | `CameraGimbal.h` (Camera class) | `include/drone/perception/camera_model.hpp` |
| clustering | `clustering.h/cpp` | `include/drone/perception/clustering.hpp` |

Key changes:
- Camera capture is direct V4L2/OpenCV (no ROS topic, no network)
- TensorRT inference from cv_cpp (C++, CUDA, zero-copy GPU buffers)
- Detection filter from drone's Yolo.h (Kalman filter)
- Airdrop decision logic from cv_py moves to Phase 6 (mission/airdrop)
- Break clustering → OffboardControl circular dependency

---

## Phase 4 — Drivers (MAVROS interface) `[DONE]`

| Module | Legacy Source | Target |
|---|---|---|
| node_base | `OffboardControl_Base.h/cpp` | `include/drone/drivers/` |
| motors | `Motors.h/cpp` | `include/drone/drivers/motors.hpp` |
| inertial_nav | `InertialNav.h/cpp` | `include/drone/drivers/inertial_nav.hpp` |
| servo | `ServoController.h` | `include/drone/drivers/servo.hpp` |
| gimbal | `CameraGimbal.h` (CameraGimbal class) | `include/drone/drivers/gimbal.hpp` |

Tasks:
- Make InertialNav fields private
- Replace raw back-pointers with shared_ptr or interfaces
- Merge servo logic from cv_py/servo_controller.py

---

## Phase 5 — Position Control `[DONE]`

| Module | Legacy Source | Target |
|---|---|---|
| pos_control | `PosControl.h/cpp` | `include/drone/control/pos_control.hpp` |

Tasks:
- Move PID constants to config
- Consider splitting into velocity/position/trajectory sub-controllers
- Extract fuzzy PID rules to YAML

---

## Phase 6 — State Machine & Mission Logic `[DONE]`

Break the god class. Merge airdrop decision logic from cv_py.

| Module | Source | Target |
|---|---|---|
| state_machine | `StateMachine.h/cpp` | `include/drone/mission/state_machine.hpp` |
| takeoff | `OffboardControl.cpp` | `include/drone/mission/takeoff.hpp` |
| airdrop | `OffboardControl.cpp` + `cv_py/detect.py` (lines 456-548) | `include/drone/mission/airdrop.hpp` |
| recon | `OffboardControl.cpp` | `include/drone/mission/recon.hpp` |
| landing | `OffboardControl.cpp` | `include/drone/mission/landing.hpp` |
| drone_node | `main.cpp` + `OffboardControl.h` | `include/drone/mission/drone_node.hpp` |

Key changes:
- Airdrop servo decision logic (from cv_py) consolidated into airdrop handler
- drone_node composes: camera_driver + detector + filter + control + mission
- Single main.cpp, single executable

---

## Phase 7 — Config & Integration `[DONE]`

- [x] main.cpp entry point + launch file
- [x] TensorRT model path + camera device config
- [x] Engine build tool (scripts/build_engine.sh)
- [x] Gazebo SITL simulation (scripts/sim.sh)
- [x] TargetTracker + catch_target visual servo
- [x] Detector integration in airdrop and landing handlers

---

## Phase 8 — Testing & CI `[DONE]`

- [x] GoogleTest (ament_cmake_gtest) with 22 test suites, 251 tests
- [x] E2E system test (DroneNode full construction + timer callback)
- [x] Integration tests (detection pipeline, PID convergence, config loading)
- [x] GitHub Actions CI (format check + build/test on ROS Humble)
- [x] clang-format + clang-tidy (warning-only) checks
- [x] Doxygen comments on all headers and sources
- [x] HDU-DXY-Team MPL-2.0 copyright headers
- [x] docs/ARCHITECTURE.md + docs/USAGE.md
