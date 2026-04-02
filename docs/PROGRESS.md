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
| Create src/utils/ | pending | |
| Extract math_utils | pending | |
| Extract readyaml | pending | |
| Extract timer | pending | |
| Extract keyboard helpers | pending | |
| Extract fuzzy_pid | pending | |
| Extract autotune | pending | |
| Extract trajectory_generator | pending | |
| Remove dead code (AC_WPNav, .bak files) | pending | |
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
