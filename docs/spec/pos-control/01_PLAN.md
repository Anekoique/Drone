# `pos-control` PLAN `01`

> Status: Draft
> Feature: `pos-control`
> Iteration: `01`
> Owner: Planner
> Dependencies:
>
> - Phase 2 math/PID infrastructure (complete)
> - Phase 4 drivers: InertialNav, NodeBase, Servo, Gimbal (complete)
> - `drone::PID`, `drone::BasicPID`, `drone::FuzzyPID`, `drone::TrajectoryGenerator`
> - Round 00 Review: `00_REVIEW.md`

---

## Log

### Review Adjustments

Round 00 was blocked with 5 HIGH, 2 MEDIUM, 1 LOW findings.

### Response Matrix

| ID | Severity | Resolution |
|---|---|---|
| R-001 | HIGH | Fixed: FuzzyPID already copies data internally via `std::vector`. Added `FuzzyConfig` struct with owned arrays as the long-lived config holder. YAML schema now shows all 8 controller parameter sets. |
| R-002 | HIGH | Fixed: Added `send_attitude()` publisher and `send_velocity_timed()` to MavrosCommander. Triage table updated. |
| R-003 | HIGH | Fixed: CascadeController::Config now has separate `dt_outer` and `dt_inner` fields matching legacy behavior. |
| R-004 | HIGH | Fixed: TrajectoryController returns `std::pair<Eigen::Vector4f, bool>` (waypoint, done). No VelocityController dependency. PosControl chains the result. |
| R-005 | HIGH | Fixed: Explicit per-method state structs added to TrajectoryController and PosControl. Reset protocol documented. |
| R-006 | MEDIUM | Fixed: `input_pos_xyz_yaw_without_vel` dropped with justification. `trajectory_setpoint` (relative) added as `setpoint_relative()`. |
| R-007 | MEDIUM | Fixed: New `drone_control` CMake target (non-ROS) + control sources added to `drone_drivers` for ROS-coupled parts. |
| R-008 | LOW | Improved: V-IT-2 uses mock InertialNav with injected positions. V-UT-12 uses inline FuzzyPID config fixture. |

### Trade-off Responses

| ID | Response |
|---|---|
| TR-1 | Adopted: Waypoint-returning API for TrajectoryController |
| TR-2 | Adopted: `drone_control` target for non-ROS control code. MavrosCommander and PosControl facade stay in `drone_drivers`. |

---

## Spec

### Goals

- G-1: Decompose PosControl into focused, single-responsibility modules
- G-2: Move all hardcoded PID gains and limits to YAML configuration
- G-3: Extract fuzzy PID rule base and membership functions to YAML
- G-4: Separate MAVROS command publishing from control computation
- G-5: Preserve all legacy control behaviors (simple PID, cascade PID, trajectory, attitude)
- G-6: Achieve 80%+ unit test coverage on control computation
- G-7: Keep the public API usable by the future mission state machine (Phase 6)

### Non-goals

- NG-1: Implementing the mission state machine (Phase 6)
- NG-2: Changing tuned PID gains or control behavior
- NG-3: Adding new control modes not present in legacy
- NG-4: Migrating auto-tune (low priority, rarely used in competition)
- NG-5: Migrating `input_pos_xyz_yaw_without_vel` — the velocity-feedback path
  (`input_pos_xyz_yaw`) is always preferred; the without-vel variant was only used
  in a commented-out call site
- NG-6: Migrating `trajectory_setpoint_world` PID::Defaults overload — no known
  call site; per-mission gain tuning should use YAML config profiles instead

### Architecture

```
include/drone/control/
  pos_control.hpp            # Facade: composes controllers + commander
  velocity_controller.hpp    # Position → velocity (simple PID path)
  cascade_controller.hpp     # Position → velocity → accel (cascade PID path)
  trajectory_controller.hpp  # S-curve + circular trajectory, returns waypoints
  mavros_commander.hpp       # All MAVROS publish logic, isolated from control
  limits.hpp                 # Limits struct + YAML loading
  yaw_utils.hpp              # Yaw wrapping/normalization utilities

src/control/
  pos_control.cpp
  velocity_controller.cpp
  cascade_controller.cpp
  trajectory_controller.cpp
  mavros_commander.cpp
  yaw_utils.cpp

config/
  pos_control.yaml           # PID gains, limits, fuzzy rules (all 8 controllers)

test/
  test_velocity_controller.cpp
  test_cascade_controller.cpp
  test_trajectory_controller.cpp
  test_yaw_utils.cpp
  test_limits.cpp
```

#### CMake Target Layout

```
drone_utils          (existing, no ROS)
  └─ drone_control   (NEW, no ROS — links drone_utils)
       │  velocity_controller.cpp
       │  cascade_controller.cpp
       │  trajectory_controller.cpp
       │  yaw_utils.cpp
       └─ drone_drivers (existing, has ROS — links drone_control)
            │  mavros_commander.cpp  (NEW, added to drone_drivers)
            │  pos_control.cpp       (NEW, added to drone_drivers)
            │  ...existing driver sources...
```

Unit tests for `drone_control` link only `drone_control` (no rclcpp).

### Invariants

- I-1: Control computation modules (`drone_control` target) have NO ROS dependencies
- I-2: Only `MavrosCommander` and `PosControl` (in `drone_drivers`) depend on rclcpp
- I-3: All PID gains loaded from YAML at construction; no hardcoded defaults in code
- I-4: FuzzyPID rule base loaded from YAML, not embedded in constructor
- I-5: `InertialNav` accessed through const reference getters only
- I-6: No static local variables — all inter-call state is in explicit state structs
- I-7: Immutable config after construction; runtime changes via explicit set methods

### Data Structure

#### File Migration Triage

| Legacy Element | Action | Target |
|---|---|---|
| `PosControl::PosControl()` (constructor) | Split | Config → YAML; publishers → `MavrosCommander`; fuzzy init → `FuzzyConfig` |
| `input_pos_xyz()` | Move | `VelocityController::compute()` |
| `input_pos_xyz_yaw()` | Move | `VelocityController::compute_with_yaw()` |
| `input_pos_xyz_yaw_without_vel()` | Drop | NG-5: velocity-feedback path always preferred; only commented-out usage |
| `input_pos_vel_1_xyz_yaw()` | Move | `CascadeController::compute()` (position stage) |
| `input_pos_vel_xyz_yaw()` | Move | `CascadeController::compute()` (velocity stage) |
| `publish_setpoint_raw()` | Move | `MavrosCommander::publish_setpoint_raw()` |
| `publish_setpoint_raw_global()` | Move | `MavrosCommander::publish_setpoint_raw_global()` |
| `send_velocity_command()` | Move | `MavrosCommander::send_velocity()` |
| `send_velocity_command_with_time()` | Move | `MavrosCommander::send_velocity_timed()` |
| `send_local_setpoint_command()` | Move | `MavrosCommander::send_position()` |
| `send_accel_command()` | Move | `MavrosCommander::send_acceleration()` |
| `setpoint_raw_attitude_publisher_` | Move | `MavrosCommander::send_attitude()` |
| `local_setpoint_command()` | Move | `PosControl::go_to_position()` with `GoToState` |
| `publish_setpoint_world()` | Move | `PosControl::hold_position()` with `HoldState` |
| `trajectory_setpoint()` (relative) | Move | `TrajectoryController::setpoint_relative()` |
| `trajectory_setpoint_world()` | Move | `TrajectoryController::setpoint_world()` |
| `trajectory_circle()` | Move | `TrajectoryController::circle()` with `CircleState` |
| `trajectory_generator_world()` | Move | `TrajectoryController::generator_world()` with `GeneratorState` |
| Fuzzy rule arrays (constructor) | Extract | `config/pos_control.yaml` via `FuzzyConfig` |
| 21 #define PID constants | Extract | `config/pos_control.yaml` |
| `Limits_t` struct | Move | `limits.hpp` as `drone::control::Limits` |
| `trajectory_setpoint_world(... PID::Defaults ...)` | Drop | NG-6: PID override overload has no known call site; gain tuning is per-config |
| `auto_tune()` | Defer | Phase 8 (NG-4) |
| `publish_statistic()` | Drop | PAL statistics not used in competition |
| Yaw wrapping logic (lines 239-250) | Extract | `yaw_utils.hpp` |

#### FuzzyPID Ownership Model

`FuzzyPID::Params` contains raw pointers (`const float*`, `const float(*)[7]`).
The refactored `FuzzyPID::init()` copies these into internal `std::vector` members
(fuzzy_pid.hpp lines 65-66), so the input pointers need only be valid during
construction. The ownership model:

```cpp
/// Holds YAML-loaded fuzzy config with owned arrays.
/// Lifetime: lives in PosControl, outlives FuzzyPID init calls.
struct FuzzyConfig {
  std::vector<float> mf_params;                       // 4 * kQfDefault floats
  std::vector<std::array<float, kQfDefault>> rules;   // 21 rows of 7

  struct ControllerParams {
    float max_error;
    float max_delta_error;
  };
  std::array<ControllerParams, 8> controllers;        // 8 per-controller configs

  /// Build all 8 FuzzyPID::Params pointing into owned data.
  /// FuzzyPID::init() expects a contiguous Params[8] array — it reads
  /// fuzzy_params[0..control_id_count-1] for per-controller max_error/max_delta_error.
  /// Pointers are valid while FuzzyConfig lives.
  std::array<FuzzyPID::Params, 8> to_params_array() const;
};

FuzzyConfig load_fuzzy_config(const YAML::Node & yaml);
```

YAML schema for all 8 controllers:

```yaml
fuzzy:
  mf_params: [...]  # 28 floats (4 * kQfDefault)
  rule_base:         # 21 rows of 7
    - [NB, NB, NM, NM, NS, ZO, ZO]
    - ...
  controllers:
    - { max_error: 1.5, max_delta_error: 0.6 }   # 0: x-position
    - { max_error: 1.5, max_delta_error: 0.6 }   # 1: y-position
    - { max_error: 1.5, max_delta_error: 0.6 }   # 2: z-position
    - { max_error: 3.14, max_delta_error: 1.0 }  # 3: yaw
    - { max_error: 2.0, max_delta_error: 0.6 }   # 4: vx cascade
    - { max_error: 2.0, max_delta_error: 0.6 }   # 5: vy cascade
    - { max_error: 1.0, max_delta_error: 0.6 }   # 6: vz cascade
    - { max_error: 100.0, max_delta_error: 100.0 } # 7: generic
```

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

Limits load_limits(const YAML::Node & yaml);

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
    float dt);

  /// Compute with fuzzy gain adaptation
  Eigen::Vector4f compute_fuzzy(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector3f & pos_target,
    float yaw_current, float yaw_target,
    float dt, FuzzyPID & fuzzy);

  void set_limits(const Limits & limits);
  void reset_pid();

private:
  PID pid_x_, pid_y_, pid_z_, pid_yaw_;
  Limits limits_;
};

}  // namespace drone::control

// --- cascade_controller.hpp ---
namespace drone::control {

class CascadeController {
public:
  struct Config {
    PID::Defaults pid_px, pid_py, pid_pz;
    PID::Defaults pid_vx, pid_vy, pid_vz;
    Limits limits;
    float dt_outer = 0.1f;   // position → velocity loop timestep
    float dt_inner = 1.0f;   // velocity → acceleration loop timestep
  };

  explicit CascadeController(const Config & config);

  /// Full cascade: position → velocity → acceleration command
  Eigen::Vector4f compute(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector3f & pos_target,
    const Eigen::Vector4f & vel_current,
    float yaw_current, float yaw_target);

  void set_limits(const Limits & limits);
  void reset_pid();

private:
  PID pid_px_, pid_py_, pid_pz_;
  PID pid_vx_, pid_vy_, pid_vz_;
  Limits limits_;
  float dt_outer_;
  float dt_inner_;
};

}  // namespace drone::control

// --- trajectory_controller.hpp ---
namespace drone::control {

/// Per-method state structs (replaces static locals from legacy)

struct CircleState {
  float angle = 0.0f;
  bool active = false;
};

struct SetpointState {
  Eigen::Vector4f cached_target = Eigen::Vector4f::Zero();
  bool initialized = false;
};

struct GeneratorState {
  Eigen::Vector3f start_pos = Eigen::Vector3f::Zero();
  Eigen::Vector4f cached_target = Eigen::Vector4f::Zero();
  bool initialized = false;
  int count = 0;
};

class TrajectoryController {
public:
  explicit TrajectoryController(const Limits & limits);

  /// S-curve trajectory: returns {waypoint, done}
  std::pair<Eigen::Vector4f, bool> setpoint_world(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector4f & pos_target,
    float yaw_current, float dt);

  /// Relative-frame trajectory: target is offset from current
  std::pair<Eigen::Vector4f, bool> setpoint_relative(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector4f & pos_offset,
    float yaw_current, float dt);

  /// Circular trajectory: returns {waypoint, done}
  std::pair<Eigen::Vector4f, bool> circle(
    float a, float b, float height,
    float angular_vel, float yaw, float dt);

  /// S-curve via TrajectoryGenerator: returns {waypoint, done}
  std::pair<Eigen::Vector4f, bool> generator_world(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector4f & pos_target,
    float yaw_current, float dt);

  /// Reset methods — must be called by PosControl when issuing a new goal
  void reset_setpoint();
  void reset_circle();
  void reset_generator();
  void reset_all();

private:
  Limits limits_;
  std::unique_ptr<TrajectoryGenerator> generator_;

  SetpointState setpoint_state_;
  SetpointState relative_state_;
  CircleState circle_state_;
  GeneratorState generator_state_;
};

}  // namespace drone::control

// --- mavros_commander.hpp ---
namespace drone::control {

class MavrosCommander {
public:
  explicit MavrosCommander(rclcpp::Node & node, const std::string & ns);

  void send_velocity(const Eigen::Vector4f & vel);
  void send_position(const Eigen::Vector4f & pos_yaw);
  void send_acceleration(const Eigen::Vector4f & accel);
  void send_attitude(const Eigen::Vector4f & attitude_thrust);
  void publish_setpoint_raw(const Eigen::Vector4f & pos, const Eigen::Vector4f & vel);
  void publish_setpoint_raw_global(double lat, double lon, double alt, float yaw);

  /// Timed velocity: sends vel for duration seconds, returns true when done.
  /// Returns true and resets timed_active_ when duration has elapsed.
  /// A subsequent call begins a new timed command.
  bool send_velocity_timed(const Eigen::Vector4f & vel, double duration_sec);

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_local_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr raw_global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;

  // State for send_velocity_timed (replaces static locals)
  rclcpp::Time timed_start_;
  bool timed_active_ = false;
};

}  // namespace drone::control

// --- pos_control.hpp ---
namespace drone::control {

/// Per-method state for facade methods (replaces static locals)
struct GoToState {
  bool first_call = true;
};

struct HoldState {
  Eigen::Vector3f start_pos = Eigen::Vector3f::Zero();
  bool initialized = false;
};

class PosControl {
public:
  PosControl(rclcpp::Node & node, InertialNav & inav, const std::string & config_path);

  /// Go to position, returns true when within accuracy
  bool go_to_position(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Hold position using PID, returns true when stable
  bool hold_position(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Trajectory to position (S-curve interpolation)
  bool trajectory_to(const Eigen::Vector4f & target);

  /// Trajectory to relative offset
  bool trajectory_relative(const Eigen::Vector4f & offset);

  /// Circular trajectory
  bool circle(float a, float b, float height, float angular_vel, float yaw);

  /// S-curve via TrajectoryGenerator
  bool trajectory_generator(const Eigen::Vector4f & target);

  /// Set control timestep
  void set_dt(float dt);

  /// Reset all controllers and state
  void reset();

  /// Reset trajectory state (call before new goal)
  void new_goal();

  /// Access sub-controllers for advanced use
  VelocityController & velocity_controller();
  CascadeController & cascade_controller();
  MavrosCommander & commander();

  /// Limits management
  void set_limits(const Limits & limits);
  void reset_limits();
  Limits get_limits() const;

  /// Get current time from ROS clock
  double get_time() const;

private:
  rclcpp::Node & node_;
  InertialNav & inav_;

  VelocityController vel_ctrl_;
  CascadeController cascade_ctrl_;
  TrajectoryController traj_ctrl_;
  MavrosCommander commander_;
  FuzzyPID fuzzy_pid_;
  FuzzyConfig fuzzy_config_;  // Owns fuzzy data, outlives FuzzyPID

  Limits default_limits_;
  float dt_ = 0.1f;
  float default_accuracy_ = 0.05f;
  float default_yaw_accuracy_ = 0.1f;

  // Per-method state (replaces static locals)
  GoToState goto_state_;
  HoldState hold_state_;
};

}  // namespace drone::control
```

### Constraints

- C-1: `VelocityController`, `CascadeController`, `TrajectoryController`, `yaw_utils`
  must be testable without ROS — no rclcpp dependency, in `drone_control` target
- C-2: YAML config schema must be backward-compatible with legacy `pos_config.yaml`
- C-3: All velocity/acceleration outputs must be clamped to `Limits` before publishing
- C-4: Yaw error must handle wrapping at +/-pi correctly
- C-5: `MavrosCommander` must use the same MAVROS topic names as legacy
- C-6: `PosControl` facade must not expose internal PID objects directly
- C-7: CascadeController must use separate dt_outer and dt_inner matching legacy
- C-8: TrajectoryController state must be explicitly reset via `new_goal()` before
  reuse — no implicit reset-on-call magic

---

## Implement

### Execution Flow

#### Main Path: go_to_position()

```
PosControl::go_to_position(target, accuracy)
  ├─ Read pos_current, yaw_current from InertialNav
  ├─ Check distance to target < accuracy → reset goto_state_, return true
  ├─ If goto_state_.first_call: publish position via commander, set first_call=false
  ├─ VelocityController::compute(pos_current, target.head<3>(), yaw, target.w(), dt)
  │   ├─ PID update for x, y, z, yaw (uses yaw_error for wrapping)
  │   ├─ Clamp to Limits
  │   └─ Return Vector4f velocity command
  └─ MavrosCommander::send_velocity(velocity)
```

#### Trajectory Path: trajectory_to()

```
PosControl::trajectory_to(target)
  ├─ Read pos_current from InertialNav
  ├─ auto [waypoint, done] = TrajectoryController::setpoint_world(pos_current, target, ...)
  ├─ VelocityController::compute(pos_current, waypoint.head<3>(), ...)
  ├─ MavrosCommander::send_velocity(velocity)
  └─ If done: return true
```

#### Reset Protocol

```
PosControl::new_goal()
  ├─ traj_ctrl_.reset_all()
  ├─ goto_state_ = GoToState{}
  └─ hold_state_ = HoldState{}
```

Mission layer (Phase 6) must call `new_goal()` before switching between different
position targets or trajectory modes.

### Implementation Plan

#### Step 1: Create `limits.hpp` and `yaw_utils.hpp/cpp`

- `drone::control::Limits` struct with defaults matching legacy constants
- `load_limits()` from YAML::Node
- `normalize_yaw()` and `yaw_error()` free functions
- Files: `include/drone/control/limits.hpp`, `include/drone/control/yaw_utils.hpp`,
  `src/control/yaw_utils.cpp`

#### Step 2: Create `velocity_controller.hpp/cpp`

- Port `input_pos_xyz()` and `input_pos_xyz_yaw()` logic
- Uses `drone::PID` for x, y, z, yaw axes
- `compute()` for standard path, `compute_fuzzy()` for fuzzy adaptation
- All state in members; `reset_pid()` clears integrators
- Files: `include/drone/control/velocity_controller.hpp`,
  `src/control/velocity_controller.cpp`

#### Step 3: Create `cascade_controller.hpp/cpp`

- Port `input_pos_vel_1_xyz_yaw()` and `input_pos_vel_xyz_yaw()`
- Two-stage PID: position PIDs use `dt_outer`, velocity PIDs use `dt_inner`
- `compute()` runs both stages and returns acceleration command
- Files: `include/drone/control/cascade_controller.hpp`,
  `src/control/cascade_controller.cpp`

#### Step 4: Create `trajectory_controller.hpp/cpp`

- State structs: `CircleState`, `SetpointState`, `GeneratorState`
- `setpoint_world()` returns `{waypoint, done}` — uses S-curve interpolation
- `setpoint_relative()` converts offset to absolute then delegates
- `circle()` accumulates angle in `CircleState`, returns next point on circle
- `generator_world()` uses `TrajectoryGenerator` for S-curve, tracks state in `GeneratorState`
- `reset_*()` methods zero each state struct
- Files: `include/drone/control/trajectory_controller.hpp`,
  `src/control/trajectory_controller.cpp`

#### Step 5: Create `mavros_commander.hpp/cpp`

- 6 publishers matching legacy topic names:
  - `setpoint_velocity/cmd_vel` (TwistStamped)
  - `setpoint_position/local` (PoseStamped)
  - `setpoint_raw/local` (PositionTarget)
  - `setpoint_raw/global` (GlobalPositionTarget)
  - `setpoint_accel/accel` (Vector3Stamped)
  - `setpoint_raw/attitude` (AttitudeTarget)
- `send_velocity_timed()` with member state (replaces static start time)
- Files: `include/drone/control/mavros_commander.hpp`,
  `src/control/mavros_commander.cpp`

#### Step 6: Create `config/pos_control.yaml`

- All PID gains (10 sets matching legacy #defines)
- Limits (speed, acceleration)
- Fuzzy config: mf_params, rule_base, 8 controller parameter sets
- Values copied exactly from legacy constants

#### Step 7: Create `pos_control.hpp/cpp` facade

- `FuzzyConfig` struct with owned arrays + `to_params()` method
- `load_fuzzy_config()` from YAML
- Constructor loads YAML, builds all sub-controllers
- High-level methods: `go_to_position`, `hold_position`, `trajectory_to`,
  `trajectory_relative`, `circle`, `trajectory_generator`
- `new_goal()` resets all trajectory and method state
- Files: `include/drone/control/pos_control.hpp`, `src/control/pos_control.cpp`

#### Step 8: Update CMakeLists.txt

- Add `drone_control` target:
  ```cmake
  add_library(drone_control
    src/control/velocity_controller.cpp
    src/control/cascade_controller.cpp
    src/control/trajectory_controller.cpp
    src/control/yaw_utils.cpp
  )
  target_link_libraries(drone_control PUBLIC drone_utils)
  ```
- Add control sources to `drone_drivers`:
  ```cmake
  # append to drone_drivers sources:
  src/control/mavros_commander.cpp
  src/control/pos_control.cpp
  ```
- Update `drone_drivers` to link `drone_control`:
  ```cmake
  target_link_libraries(drone_drivers PUBLIC drone_control drone_utils)
  ```
- Install `drone_control` target

#### Step 9: Write unit tests

- `test_yaw_utils.cpp` — link `drone_control`
  - V-UT-1: normalize_yaw handles [-2pi, 2pi]
  - V-UT-2: yaw_error shortest path for all quadrant pairs
- `test_velocity_controller.cpp` — link `drone_control`
  - V-UT-3: zero output at target
  - V-UT-4: output respects limits
  - V-UT-5: moves toward target (correct sign)
  - V-UT-12: compute_fuzzy adapts gains (inline FuzzyPID config fixture)
- `test_cascade_controller.cpp` — link `drone_control`
  - V-UT-6: zero output at target with zero velocity
  - V-UT-7: output respects limits
  - V-UT-13: inner/outer dt produce different responses vs single dt
- `test_trajectory_controller.cpp` — link `drone_control`
  - V-UT-8: setpoint_world progresses toward target
  - V-UT-9: circle produces circular motion
  - V-UT-14: reset_circle starts new circle from angle 0
  - V-UT-15: setpoint_relative converts to absolute correctly
- `test_limits.cpp` — link `drone_control`
  - V-UT-10: Limits loads from YAML
  - V-UT-11: Fuzzy rules load from YAML and match legacy values

### Trade-offs

#### T-1: Single `drone` umbrella target vs separate targets

- Decision: Separate `drone_control` (non-ROS) + extend `drone_drivers` (ROS)
- Rationale: Enforces I-1 at the build level. Unit tests link only `drone_control`.

#### T-2: Fuzzy rules in YAML vs code

- Decision: YAML with `FuzzyConfig` ownership struct
- Rationale: Rule tables are data. YAML allows tuning without recompilation.
  `FuzzyConfig` owns the arrays; `to_params()` produces temporary `Params` valid
  only during `FuzzyPID::init()`, which copies data internally.

#### T-3: Cascade controller included vs deferred

- Decision: Include with separate dt_outer/dt_inner matching legacy
- Rationale: Preserves option for cascade mode. Small, testable code.

#### T-4: Auto-tune inclusion

- Decision: Defer to Phase 8
- Rationale: Rarely used, hardware-dependent, heavy use of statics.

---

## Validation

### Unit Tests

- V-UT-1: `normalize_yaw` handles [-2pi, 2pi] range correctly
- V-UT-2: `yaw_error` returns shortest path for all quadrant pairs
- V-UT-3: `VelocityController::compute` produces zero output at target
- V-UT-4: `VelocityController::compute` output respects limits
- V-UT-5: `VelocityController::compute` moves toward target (correct sign)
- V-UT-6: `CascadeController::compute` produces zero output at target+zero velocity
- V-UT-7: `CascadeController::compute` output respects limits
- V-UT-8: `TrajectoryController::setpoint_world` progresses toward target
- V-UT-9: `TrajectoryController::circle` produces circular motion
- V-UT-10: `Limits` loads correctly from YAML
- V-UT-11: Fuzzy rules load from YAML and match legacy values
- V-UT-12: `VelocityController::compute_fuzzy` adapts gains (inline config fixture)
- V-UT-13: `CascadeController` with different dt_outer/dt_inner produces different
  response than with equal dt values
- V-UT-14: `TrajectoryController::reset_circle` starts new circle from angle 0
- V-UT-15: `TrajectoryController::setpoint_relative` converts offset to absolute

### Integration Tests

- V-IT-1: `PosControl` constructs successfully with valid YAML config
- V-IT-2: `PosControl::go_to_position` converges in mock simulation loop
  (mock InertialNav with injected position updates, mock MavrosCommander captures messages)
- V-IT-3: All control headers compile standalone (no hidden dependencies)
- V-IT-4: `new_goal()` properly resets all state for trajectory reuse

### Acceptance Mapping

| Goal | Validation |
|------|------------|
| G-1 Decompose | Architecture: 7 focused files, each < 400 lines |
| G-2 YAML gains | V-UT-10, V-UT-11, config/pos_control.yaml |
| G-3 Fuzzy YAML | V-UT-11, V-UT-12, FuzzyConfig ownership |
| G-4 Separate publish | MavrosCommander isolated, I-1 enforced by CMake target |
| G-5 Preserve behavior | V-UT-3..9, V-UT-13..15, V-IT-2, triage table complete |
| G-6 Test coverage | 15 unit tests + 4 integration tests |
| G-7 Mission-ready API | PosControl facade + new_goal() reset protocol |
