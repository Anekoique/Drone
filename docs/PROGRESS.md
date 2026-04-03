# Refactoring Progress

Tracks completion status for each phase in [ROADMAP.md](./ROADMAP.md).

---

## Phase 0 — Project Skeleton

| Task | Status | Date | Notes |
|---|---|---|---|
| README, CLAUDE.md, AGENTS.md | done | 2026-04-02 | |
| docs/PROBLEM.md | done | 2026-04-02 | Competition rules translated |
| docs/DEPENDENCIES.md | done | 2026-04-02 | Full dependency analysis with MAVROS topic graph |
| scripts/deps/ | done | 2026-04-02 | common.sh + local.sh + raspi.sh |
| docs/ROADMAP.md | done | 2026-04-02 | 8-phase plan |
| CMakeLists.txt | done | 2026-04-02 | C++20, ament_cmake, config install |
| package.xml | done | 2026-04-02 | Format 3, ament_cmake export |
| config/ migration | done | 2026-04-02 | 5 files renamed, byte-identical values |
| .clang-format | done | 2026-04-02 | ament/autoware conventions, Google base |
| src/.gitkeep | done | 2026-04-02 | Empty src/ ready for Phase 1 |

---

## Phase 1 — Pure Utilities

| Task | Status | Notes |
|---|---|---|
| Directory structure (include/drone/, src/) | done | include/drone/{utils,control,trajectory} |
| Extract readyaml | done | ConfigLoader with ament installed layout |
| Extract timer | done | drone::Timer in utils/timer.hpp |
| Extract keyboard helpers | done | drone::kbhit/getch in utils/keyboard.hpp |
| Extract rotate | done | drone::rotate_2d in utils/rotate.hpp |
| Extract fuzzy_pid | done | std::vector replaces malloc, clamp bug fixed, off-by-one fixed |
| Extract autotune | done | Modern enums, Tu ms→s unit fix |
| Extract trajectory_generator | done | drone::TrajectoryGenerator |
| CMakeLists + package.xml | done | Exported lib with PUBLIC yaml-cpp/Eigen3 |
| Code review fixes | done | 9 HIGH issues resolved |
| Docker build verification | done | 0 errors, 0 failures |
| Extract math_utils | deferred | Migrates with full math layer |
| Unit tests | pending | |

---

## Phase 2 — Math & Sensor Types

| Task | Status | Notes |
|---|---|---|
| Create include/drone/math/ + src/math/ | done | |
| Extract utils (was math_utils + math) | done | Predicates, constrain_float→std::clamp, safe_sqrt |
| Extract types (was math) | done | Eigen helpers: sq, norm, rotate_xy, kinematic_limit |
| Extract PID | done | Removed duplicated fns, static locals→members, error ordering fix |
| Extract BasicPID (was MYPID) | done | Renamed, std::clamp, ConfigLoader, exception handling |
| Extract SCurve | done | Eigen:: prefix, std::min/max, std::isfinite, index guards |
| Remove `using namespace Eigen` | done | Zero occurrences |
| Replace MIN/MAX macros | done | std::min/std::max |
| Code review fixes (6 HIGH) | done | constrain_float, PID statics, error ordering, segment guard |
| Docker build verification | done | 0 errors, 0 warnings, 0 test failures |
| Unit tests (Phase 1+2) | done | timer, rotate, fuzzy_pid, math_utils, pid |

---

## Phase 3 — Perception

| Task | Status | Notes |
|---|---|---|
| Directory structure | done | include/drone/perception/ + detector/ subdirectory |
| Detection types + adapter | done | Detection, TargetSample, Target, to_detection2d_array() |
| Camera driver | done | V4L2/OpenCV capture, no ROS |
| Detection filter (Kalman) | done | Joseph form, per-class tracking, sorted by center |
| Camera model | done | pixel↔world, ENU/NED/ESD, distortion, get_position fix |
| Clustering | done | k-means with diameter, break OffboardControl dep |
| TensorRT detector headers | done | config, types, cuda_utils, preprocess, postprocess |
| TensorRT detector impl | done | detector.cpp, preprocess.cu, plugin/yolo_layer.cu |
| NMS postprocess | done | Ported from cv_cpp, removed draw functions |
| YOLO plugin | done | YoloLayerPlugin + Creator, auto-registration |
| Conditional CUDA CMake | done | check_language(CUDA) + CUDAToolkit + find_library(nvinfer) |
| drone_perception library | done | Separate target from drone_utils |
| Code review fixes (6 HIGH) | done | rotation chain, Kalman Joseph form, clustering guards |
| Unit tests (Phase 3) | done | camera_model, detection_filter, clustering |
| Docker build verification | done | 98 tests, 0 errors, 0 failures |

---

## Phase 4 — Infrastructure

| Task | Status | Notes |
|---|---|---|
| Directory structure | done | include/drone/drivers/ + src/drivers/ |
| NodeBase | done | rclcpp::Node inheritance, params, mode client |
| Motors | done | Takeoff state machine, async services, bool returns |
| InertialNav | done | Private fields, const getters, yaw from quaternion |
| Servo | done | Non-blocking fire_servo (one-shot timer, per-ID map) |
| Gimbal | done | MountControl pub/sub (legacy MAVROS) |
| Node& reference pattern | done | No shared_from_this, no raw pointers |
| drone_drivers library | done | Separate CMake target |
| Docker build verification | done | 108 tests, 0 errors, 0 failures |
| Integration tests | pending | Needs MAVROS mock infrastructure |

---

## Phase 5 — Position Control

| Task | Status | Notes |
|---|---|---|
| Create drone_control library target | done | Non-ROS: yaw_utils, velocity_controller, cascade_controller, trajectory_controller |
| VelocityController | done | Ports input_pos_xyz/input_pos_xyz_yaw, fuzzy adaptation |
| CascadeController | done | Dual timestep (dt_outer/dt_inner), ports cascade PID path |
| TrajectoryController | done | Waypoint-returning API, state structs replace static locals |
| MavrosCommander | done | All 6 publishers, send_velocity_timed, attitude support |
| PosControl facade | done | Composes all sub-controllers, FuzzyConfig ownership model |
| Move PID constants to config | done | config/pos_control.yaml with all 10 PID sets + limits |
| Extract fuzzy PID rules to YAML | done | 21x7 rule base, 28 MF params, 8 controller configs |
| PLAN/REVIEW iteration | done | 2 rounds: 00 blocked (5 HIGH), 01 approved |
| Unit tests | done | yaw_utils(14), velocity_controller(6), cascade(5), trajectory(7), limits(5) |
| Docker build verification | done | 168 tests, 0 errors, 0 failures |

---

## Phase 6 — State Machine & Mission Logic

| Task | Status | Notes |
|---|---|---|
| Create src/mission/ | pending | |
| Extract TakeoffHandler | pending | |
| Extract AirdropHandler | pending | |
| Extract ReconHandler | pending | |
| Extract LandingHandler | pending | |
| Make StateMachine generic | pending | |
| Create FrameTransforms utility | pending | |
| Move waypoints to config | pending | |
| Replace god class with MissionNode | pending | |

---

## Phase 7 — Config & Launch

| Task | Status | Notes |
|---|---|---|
| Consolidate YAML configs | pending | |
| Create launch files | pending | |
| Document all parameters | pending | |

---

## Phase 8 — Testing & CI

| Task | Status | Notes |
|---|---|---|
| GoogleTest setup | pending | |
| 80%+ coverage on core modules | pending | |
| Mock MAVROS integration tests | pending | |
| GitHub Actions CI | pending | |
| clang-format CI check | pending | |
| clang-tidy CI check | pending | |
