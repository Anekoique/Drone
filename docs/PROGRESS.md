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
| Create src/math/ | pending | |
| Extract math.h | pending | |
| Extract PID | pending | |
| Extract SCurve | pending | |
| Remove `using namespace Eigen` from headers | pending | |
| Replace #define PID constants | pending | |
| Unit tests | pending | |

---

## Phase 3 — Perception

| Task | Status | Notes |
|---|---|---|
| Create src/perception/ | pending | |
| Extract KalmanFilter2D | pending | |
| Extract detection_subscriber (YOLO) | pending | |
| Split Camera from CameraGimbal | pending | |
| Break clustering → OffboardControl circular dep | pending | |
| Unit tests | pending | |

---

## Phase 4 — Infrastructure

| Task | Status | Notes |
|---|---|---|
| Create src/drivers/ | pending | |
| Extract node_base | pending | |
| Extract motors | pending | |
| Extract inertial_nav (private fields) | pending | |
| Extract servo_controller | pending | |
| Replace raw back-pointers | pending | |
| Integration tests | pending | |

---

## Phase 5 — Position Control

| Task | Status | Notes |
|---|---|---|
| Create src/control/ | pending | |
| Split PosControl into sub-controllers | pending | |
| Move PID constants to config | pending | |
| Extract fuzzy PID rules to YAML | pending | |
| Unit tests | pending | |

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
