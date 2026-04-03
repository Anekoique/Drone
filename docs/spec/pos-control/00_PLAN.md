# `pos-control` PLAN `00`

> Status: Draft
> Feature: `pos-control`
> Iteration: `00`
> Owner: Planner
> Dependencies:
>
> - Phase 2 math/PID infrastructure (complete)
> - Phase 4 drivers: InertialNav, NodeBase, Servo, Gimbal (complete)
> - `drone::PID`, `drone::BasicPID`, `drone::FuzzyPID`, `drone::TrajectoryGenerator`

---

## Log

### Feature Introduce

Legacy `PosControl.h/cpp` (~1370 lines total) is a monolithic position controller
combining:

- 10 PID objects (4 simple + 6 cascade) with 21 hardcoded #define constants
- 8 FuzzyPID controllers with inline 21x7 rule base + 28-element MF params
- 6 MAVROS publishers for different command types
- Trajectory generation (S-curve, circular)
- Auto-tuning (Ziegler-Nichols)
- 39 public methods mixing control logic with ROS transport

Phase 5 migrates this into clean, testable sub-modules under `drone::control`.

### Changes from Previous Round

- N/A (first iteration)

---

## Spec

### Goals

- G-1: Decompose PosControl into focused, single-responsibility modules
- G-2: Move all hardcoded PID gains and limits to YAML configuration
- G-3: Extract fuzzy PID rule base and membership functions to YAML
- G-4: Separate MAVROS command publishing from control computation
- G-5: Preserve all legacy control behaviors (simple PID, cascade PID, trajectory)
- G-6: Achieve 80%+ unit test coverage on control computation
- G-7: Keep the public API usable by the future mission state machine (Phase 6)

### Non-goals

- NG-1: Implementing the mission state machine (Phase 6)
- NG-2: Changing tuned PID gains or control behavior
- NG-3: Adding new control modes not present in legacy
- NG-4: Migrating auto-tune (low priority, rarely used in competition)

### Architecture

```
include/drone/control/
  pos_control.hpp          # Facade: composes controllers + commander
  velocity_controller.hpp  # Position → velocity (simple PID path)
  cascade_controller.hpp   # Position → velocity → accel (cascade PID path)
  trajectory_controller.hpp # S-curve + circular trajectory wrappers
  mavros_commander.hpp     # All MAVROS publish logic, isolated from control
  limits.hpp               # Limits_t struct + defaults
  yaw_utils.hpp            # Yaw wrapping/normalization utilities

src/control/
  pos_control.cpp
  velocity_controller.cpp
  cascade_controller.cpp
  trajectory_controller.cpp
  mavros_commander.cpp
  yaw_utils.cpp

config/
  pos_control.yaml         # PID gains, limits, fuzzy rules

test/
  test_velocity_controller.cpp
  test_cascade_controller.cpp
  test_trajectory_controller.cpp
  test_yaw_utils.cpp
  test_limits.cpp
```

### Invariants

- I-1: Control computation modules have NO ROS dependencies — pure math
- I-2: Only `MavrosCommander` and `PosControl` facade depend on rclcpp
- I-3: All PID gains loaded from YAML at construction; no hardcoded defaults in code
- I-4: FuzzyPID rule base loaded from YAML, not embedded in constructor
- I-5: `InertialNav` accessed through const reference getters only
- I-6: No static local variables — all state is member variables
- I-7: Immutable config after construction; runtime changes via explicit set methods

### Data Structure

#### File Migration Triage

| Legacy Element | Action | Target |
|---|---|---|
| `PosControl::PosControl()` (constructor, 150 lines) | Split | Config loading → YAML; publisher setup → `MavrosCommander` |
| `input_pos_xyz()` | Move | `VelocityController::compute()` |
| `input_pos_xyz_yaw()` | Move | `VelocityController::compute_with_yaw()` |
| `input_pos_vel_1_xyz_yaw()` | Move | `CascadeController::compute_position_stage()` |
| `input_pos_vel_xyz_yaw()` | Move | `CascadeController::compute_velocity_stage()` |
| `publish_setpoint_raw()` | Move | `MavrosCommander::publish_setpoint_raw()` |
| `send_velocity_command()` | Move | `MavrosCommander::send_velocity()` |
| `send_local_setpoint_command()` | Move | `MavrosCommander::send_position()` |
| `send_accel_command()` | Move | `MavrosCommander::send_acceleration()` |
| `trajectory_setpoint_world()` | Move | `TrajectoryController::setpoint_world()` |
| `trajectory_circle()` | Move | `TrajectoryController::circle()` |
| `trajectory_generator_world()` | Move | `TrajectoryController::generator_world()` |
| `local_setpoint_command()` | Move | `PosControl::go_to_position()` |
| `publish_setpoint_world()` | Move | `PosControl::hold_position()` |
| Fuzzy rule arrays (constructor) | Extract | `config/pos_control.yaml` |
| 21 #define PID constants | Extract | `config/pos_control.yaml` |
| `Limits_t` struct | Move | `limits.hpp` |
| `auto_tune()` | Defer | Phase 8 or drop (NG-4) |
| `publish_statistic()` | Drop | PAL statistics not used |
| Yaw wrapping logic (lines 239-250) | Extract | `yaw_utils.hpp` |

### API Surface

```cpp
// --- limits.hpp ---
namespace drone::control {

struct Limits {
  float speed_max_xy = 2.0f;
  float speed_max_z = 1.0f;
  float speed_max_yaw = 0.3f;
  float accel_max_xy = 1.2f;
  float accel_max_z = 1.2f;
  float accel_max_yaw = 0.0f;
};

}  // namespace drone::control

// --- yaw_utils.hpp ---
namespace drone::control {

/// Normalize angle to [-pi, pi]
float normalize_yaw(float yaw);

/// Shortest angular difference from current to target
float yaw_error(float current, float target);

}  // namespace drone::control

// --- velocity_controller.hpp ---
namespace drone::control {

/// Single-loop position → velocity controller (simple PID path)
class VelocityController {
public:
  struct Config {
    PID::Defaults pid_x, pid_y, pid_z, pid_yaw;
    Limits limits;
  };

  explicit VelocityController(const Config & config);

  /// Compute velocity command from position error
  Eigen::Vector4f compute(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector3f & pos_target,
    float yaw_current, float yaw_target,
    float dt) const;

  /// Compute with optional fuzzy gain adaptation
  Eigen::Vector4f compute_fuzzy(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector3f & pos_target,
    float yaw_current, float yaw_target,
    float dt, FuzzyPID & fuzzy) const;

  void set_limits(const Limits & limits);
  void reset_pid();

private:
  PID pid_x_, pid_y_, pid_z_, pid_yaw_;
  Limits limits_;
};

}  // namespace drone::control

// --- cascade_controller.hpp ---
namespace drone::control {

/// Cascade position → velocity → acceleration controller
class CascadeController {
public:
  struct Config {
    PID::Defaults pid_px, pid_py, pid_pz;
    PID::Defaults pid_vx, pid_vy, pid_vz;
    Limits limits;
  };

  explicit CascadeController(const Config & config);

  /// Full cascade: position error → velocity → acceleration command
  Eigen::Vector4f compute(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector3f & pos_target,
    const Eigen::Vector4f & vel_current,
    float yaw_current, float yaw_target,
    float dt) const;

  void set_limits(const Limits & limits);
  void reset_pid();

private:
  PID pid_px_, pid_py_, pid_pz_;
  PID pid_vx_, pid_vy_, pid_vz_;
  Limits limits_;
};

}  // namespace drone::control

// --- trajectory_controller.hpp ---
namespace drone::control {

class TrajectoryController {
public:
  explicit TrajectoryController(const Limits & limits);

  /// S-curve trajectory from current to target
  bool setpoint_world(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector4f & pos_target,
    float yaw_current, float dt,
    VelocityController & vel_ctrl,
    Eigen::Vector4f & velocity_out);

  /// Circular trajectory
  bool circle(
    float a, float b, float height,
    float angular_vel, float yaw,
    float dt,
    VelocityController & vel_ctrl,
    Eigen::Vector4f & velocity_out);

  /// S-curve via TrajectoryGenerator
  bool generator_world(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector4f & pos_target,
    float yaw_current, float dt,
    VelocityController & vel_ctrl,
    Eigen::Vector4f & velocity_out);

private:
  Limits limits_;
  std::unique_ptr<TrajectoryGenerator> generator_;
};

}  // namespace drone::control

// --- mavros_commander.hpp ---
namespace drone::control {

/// Encapsulates all MAVROS setpoint publishing
class MavrosCommander {
public:
  explicit MavrosCommander(rclcpp::Node & node, const std::string & ns);

  void send_velocity(const Eigen::Vector4f & vel);
  void send_position(const Eigen::Vector4f & pos_yaw);
  void send_acceleration(const Eigen::Vector4f & accel);
  void publish_setpoint_raw(const Eigen::Vector4f & pos, const Eigen::Vector4f & vel);
  void publish_setpoint_raw_global(double lat, double lon, double alt, float yaw);

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_local_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr raw_global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
};

}  // namespace drone::control

// --- pos_control.hpp ---
namespace drone::control {

/// Top-level position controller facade
class PosControl {
public:
  PosControl(rclcpp::Node & node, InertialNav & inav, const std::string & config_path);

  /// Go to position, returns true when within accuracy
  bool go_to_position(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Hold position using PID, returns true when stable
  bool hold_position(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Trajectory to position
  bool trajectory_to(const Eigen::Vector4f & target);

  /// Circular trajectory
  bool circle(float a, float b, float height, float angular_vel, float yaw);

  /// Set control timestep
  void set_dt(float dt);

  /// Reset all controllers
  void reset();

  /// Access sub-controllers for advanced use
  VelocityController & velocity_controller();
  CascadeController & cascade_controller();
  MavrosCommander & commander();

  /// Limits management
  void set_limits(const Limits & limits);
  void reset_limits();
  Limits get_limits() const;

private:
  rclcpp::Node & node_;
  InertialNav & inav_;

  VelocityController vel_ctrl_;
  CascadeController cascade_ctrl_;
  TrajectoryController traj_ctrl_;
  MavrosCommander commander_;
  FuzzyPID fuzzy_pid_;

  Limits default_limits_;
  float dt_ = 0.1f;
  float default_accuracy_ = 0.05f;
  float default_yaw_accuracy_ = 0.1f;
};

}  // namespace drone::control
```

### Constraints

- C-1: `VelocityController`, `CascadeController`, `TrajectoryController` must be
  testable without ROS — no rclcpp dependency
- C-2: YAML config schema must be backward-compatible with legacy `pos_config.yaml`
- C-3: All velocity/acceleration outputs must be clamped to `Limits` before publishing
- C-4: Yaw error must handle wrapping at +/-pi correctly
- C-5: `MavrosCommander` must use the same MAVROS topic names as legacy
- C-6: `PosControl` facade must not expose internal PID objects directly

---

## Implement

### Execution Flow

#### Main Path: go_to_position()

```
PosControl::go_to_position(target, accuracy)
  ├─ Read pos_current from InertialNav
  ├─ Check distance to target < accuracy → return true
  ├─ VelocityController::compute(pos_current, target, yaw, dt)
  │   ├─ PID update for x, y, z, yaw
  │   ├─ Clamp to Limits
  │   └─ Return Vector4f velocity command
  └─ MavrosCommander::send_velocity(velocity)
```

#### Trajectory Path: trajectory_to()

```
PosControl::trajectory_to(target)
  ├─ Read pos_current from InertialNav
  ├─ TrajectoryController::generator_world(pos_current, target, ...)
  │   ├─ TrajectoryGenerator generates S-curve waypoint
  │   ├─ VelocityController::compute(pos_current, waypoint, ...)
  │   └─ Return velocity command + done flag
  └─ MavrosCommander::send_velocity(velocity)
```

### Implementation Plan

#### Step 1: Create `limits.hpp`

- Define `drone::control::Limits` struct with default values matching legacy
- Add `load_limits(const YAML::Node &)` free function
- File: `include/drone/control/limits.hpp`

#### Step 2: Create `yaw_utils.hpp/cpp`

- `normalize_yaw(float)` — wrap to [-pi, pi]
- `yaw_error(float current, float target)` — shortest path difference
- Extract from legacy lines 239-250
- File: `include/drone/control/yaw_utils.hpp`, `src/control/yaw_utils.cpp`

#### Step 3: Create `velocity_controller.hpp/cpp`

- Port `input_pos_xyz()` and `input_pos_xyz_yaw()` logic
- Uses `drone::PID` for x, y, z, yaw axes
- Optional fuzzy gain adaptation via `compute_fuzzy()`
- All state in members, no statics
- File: `include/drone/control/velocity_controller.hpp`, `src/control/velocity_controller.cpp`

#### Step 4: Create `cascade_controller.hpp/cpp`

- Port `input_pos_vel_1_xyz_yaw()` and `input_pos_vel_xyz_yaw()`
- Two-stage PID: position PIDs output velocity targets → velocity PIDs output accel
- File: `include/drone/control/cascade_controller.hpp`, `src/control/cascade_controller.cpp`

#### Step 5: Create `trajectory_controller.hpp/cpp`

- Wrap `TrajectoryGenerator` for S-curve trajectories
- Port `trajectory_circle()` for circular paths
- Port `trajectory_setpoint_world()` logic
- File: `include/drone/control/trajectory_controller.hpp`, `src/control/trajectory_controller.cpp`

#### Step 6: Create `mavros_commander.hpp/cpp`

- Port all 6 publishers from legacy
- Each method constructs and publishes a single message type
- Takes `rclcpp::Node &` for publisher creation
- File: `include/drone/control/mavros_commander.hpp`, `src/control/mavros_commander.cpp`

#### Step 7: Create `config/pos_control.yaml`

- All PID gains (10 sets: simple x/y/z/yaw + cascade px/py/pz/vx/vy/vz)
- Limits (speed_max_xy/z/yaw, accel_max_xy/z)
- Fuzzy PID rules (rule_base 21x7, membership_functions 4x7)
- Fuzzy PID params for each of 8 controllers
- File: `config/pos_control.yaml`

#### Step 8: Create `pos_control.hpp/cpp` facade

- Compose VelocityController, CascadeController, TrajectoryController, MavrosCommander
- Load config from YAML in constructor
- Expose high-level API: go_to_position, hold_position, trajectory_to, circle
- File: `include/drone/control/pos_control.hpp`, `src/control/pos_control.cpp`

#### Step 9: Update CMakeLists.txt

- Add new source files to `drone` target
- No new library target needed — control code belongs in main `drone` target

#### Step 10: Write unit tests

- `test_yaw_utils.cpp` — normalize, error, wrapping edge cases
- `test_velocity_controller.cpp` — PID response, limit clamping, zero-error
- `test_cascade_controller.cpp` — two-stage PID, limit enforcement
- `test_trajectory_controller.cpp` — S-curve progression, circle generation
- `test_limits.cpp` — YAML loading, default values

### Trade-offs

#### T-1: Separate library target vs single target

- Decision: Keep in single `drone` target
- Rationale: Control code depends on math/PID, drivers, and utils already in `drone`.
  A separate target adds link complexity without benefit at this project size.

#### T-2: Fuzzy rules in YAML vs code

- Decision: YAML
- Rationale: Rule tables are data, not logic. YAML allows tuning without recompilation.
  The 21x7 rule base and 28-element MF params are configuration, not algorithm.

#### T-3: Cascade controller included vs deferred

- Decision: Include but mark as secondary path
- Rationale: Legacy code defines cascade methods. Even if unused in current missions,
  preserving them maintains the option. The code is small and testable.

#### T-4: Auto-tune inclusion

- Decision: Defer to Phase 8
- Rationale: Rarely used in competition, complex implementation, depends on real
  hardware. Not worth blocking Phase 5 on it.

---

## Validation

### Unit Tests

- V-UT-1: `normalize_yaw` handles [-2pi, 2pi] range correctly
- V-UT-2: `yaw_error` returns shortest path for all quadrant pairs
- V-UT-3: `VelocityController::compute` produces zero output at target
- V-UT-4: `VelocityController::compute` output respects limits
- V-UT-5: `VelocityController::compute` moves toward target (correct sign)
- V-UT-6: `CascadeController::compute` produces zero output at target with zero velocity
- V-UT-7: `CascadeController::compute` output respects limits
- V-UT-8: `TrajectoryController` progresses toward target over time
- V-UT-9: `TrajectoryController::circle` produces circular motion
- V-UT-10: `Limits` loads correctly from YAML
- V-UT-11: Fuzzy rules load from YAML and match legacy values
- V-UT-12: `VelocityController::compute_fuzzy` adapts gains based on error

### Integration Tests

- V-IT-1: `PosControl` constructs successfully with valid YAML config
- V-IT-2: `PosControl::go_to_position` converges in simulation loop
- V-IT-3: All control headers compile standalone (no hidden dependencies)

### Acceptance Mapping

| Goal | Validation |
|------|------------|
| G-1 Decompose | Architecture: 6 focused files, each < 400 lines |
| G-2 YAML gains | V-UT-10, V-UT-11, config/pos_control.yaml |
| G-3 Fuzzy YAML | V-UT-11, V-UT-12 |
| G-4 Separate publish | MavrosCommander isolated, C-1 enforced |
| G-5 Preserve behavior | V-UT-3..9, V-IT-2 |
| G-6 Test coverage | 12 unit tests + 3 integration tests |
| G-7 Mission-ready API | PosControl facade API review |
