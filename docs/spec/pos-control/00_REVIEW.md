# `pos-control` REVIEW `00`

> Status: Open
> Feature: `pos-control`
> Iteration: `00`
> Owner: Reviewer
> Target Plan: `00_PLAN.md`
> Review Scope:
>
> - Plan Correctness
> - Spec Alignment
> - Design Soundness
> - API Completeness
> - Validation Adequacy
> - Dependency Analysis

---

## Verdict

- Decision: Revisions Required
- Blocking Issues: `5`
- Non-Blocking Issues: `3`

## Summary

The plan correctly identifies the decomposition intent and gets the module
structure largely right. The separation of MAVROS publishing into
`MavrosCommander` is clean and the YAML extraction of PID gains and fuzzy rules
is the right direction. The trade-off choices are reasonable.

However, the legacy code contains several behaviors that the plan either
silently drops, misnames, or leaves semantically incorrect in the new API.
Five blocking issues were found by tracing through `PosControl.cpp`
method-by-method:

1. The `FuzzyPID` API boundary changed between legacy and the refactored
   `drone::FuzzyPID`, but the plan does not acknowledge the structural
   difference or specify how YAML loading maps to the new `Params` struct.
2. The `MavrosCommander` API surface omits the `AttitudeTarget` publisher and
   the `send_velocity_command_with_time` timed-velocity method — both present
   in the legacy class and used from mission-layer call sites.
3. The `CascadeController` `compute()` signature is incorrect: the legacy
   cascade path uses a fixed `dt_pid_p_v = 1` for the inner loop and the
   outer loop uses `dt`; the plan collapses them into a single `dt` parameter,
   which would change the inner-loop gain scaling and break existing tuning.
4. `TrajectoryController` takes a mutable `VelocityController &` as a
   parameter to `setpoint_world()`, `circle()`, and `generator_world()`,
   making `TrajectoryController` stateful on an external object. This violates
   the invariant that sub-controllers are composed inside the `PosControl`
   facade and couples the trajectory layer back into the velocity layer in a
   way that is not testable in isolation.
5. The static local variable pattern (`static bool first`) used in at least
   seven legacy methods to maintain inter-call state is listed as Invariant
   I-6 ("no static local variables") but the plan provides no substitute
   mechanism. Without a concrete state replacement design, implementors will
   either silently re-introduce statics or break the convergence logic.

---

## Findings

### R-001 `FuzzyPID YAML loading path is under-specified and structurally mismatched`

- Severity: HIGH
- Section: `Spec / Invariants I-3 I-4 / Implement Step 7`
- Type: Correctness
- Problem:
  The legacy constructor builds a `FuzzyPID::Fuzzy_params[8]` array and passes
  it as a raw pointer to `FuzzyPID(struct Fuzzy_params*)`. The refactored
  `drone::FuzzyPID` uses a different `Params` struct with `const float*
  mf_params` and `const float (*rule_base)[kQfDefault]` — both raw pointer
  fields that assume the backing arrays outlive the object.

  The plan says "Fuzzy rules load from YAML" (G-3, V-UT-11) and lists the YAML
  key structure (21x7 rule base, 28-element MF params), but it does not specify:
  (a) what concrete C++ type holds the deserialized arrays at config load time,
  (b) how the raw pointer fields in `FuzzyPID::Params` are satisfied without
  dangling after the load function returns, or
  (c) whether the 8 per-controller `Fuzzy_params` entries (with per-controller
  `max_error` and `max_delta_error` values from the legacy header, lines
  137-145) are preserved or collapsed.

  There are also 8 distinct controller slots in the legacy `Fuzzy_params` array
  (for xy x2, z, yaw, cascade-vx, vy, vz, and one extra with max 100), but
  `PosControl` only stores one `FuzzyPID fuzzy_pid_` member. The plan inherits
  this model without explaining it.
- Why it matters:
  An implementation that naively loads `mf_params` into a `std::vector<float>`
  and then passes `.data()` to `FuzzyPID::Params` will produce a dangling
  pointer as soon as the vector goes out of scope. This is an undefined
  behavior bug that will be silent in release builds and intermittent under
  address sanitizer.
- Recommendation:
  Round 01 must specify a concrete ownership model. Options: (a) embed the
  arrays in a `FuzzyConfig` struct that lives in `PosControl` and outlives the
  `FuzzyPID` object, (b) change `FuzzyPID::Params` to own its data via
  `std::vector`, or (c) make `FuzzyPID` accept a YAML node directly and own
  its own data. The YAML schema spec must also show all 8 controller parameter
  sets with their distinct `max_error` / `max_delta_error` entries.

### R-002 `MavrosCommander is missing the AttitudeTarget publisher and the timed-velocity method`

- Severity: HIGH
- Section: `Spec / Data Structure / API Surface / mavros_commander.hpp`
- Type: Completeness
- Problem:
  The legacy class has **6 MAVROS publishers**, but the plan's
  `MavrosCommander` API surface only lists 5:

  | Publisher | Legacy topic | Plan |
  |---|---|---|
  | `twist_stamped_publisher_` | `setpoint_velocity/cmd_vel` | `send_velocity()` |
  | `local_setpoint_publisher_` | `setpoint_position/local` | `send_position()` |
  | `setpoint_raw_local_publisher_` | `setpoint_raw/local` | `publish_setpoint_raw()` |
  | `setpoint_raw_global_publisher_` | `setpoint_raw/global` | `publish_setpoint_raw_global()` |
  | `setpoint_accel_publisher_` | `setpoint_accel/accel` | `send_acceleration()` |
  | `setpoint_raw_attitude_publisher_` | `setpoint_raw/attitude` | **MISSING** |

  The `mavros_msgs::msg::AttitudeTarget` publisher (`setpoint_raw/attitude`) is
  declared and constructed in the legacy header (lines 101, 317) but does not
  appear anywhere in the plan's API surface.

  Additionally, `send_velocity_command_with_time()` (legacy line 250) — which
  times a fixed-duration velocity command and then stops the drone — is also
  absent. It is a public method on the legacy class and is a distinct behavior
  that mission code can depend on.
- Why it matters:
  Omitting the attitude publisher makes `MavrosCommander` a regression: the
  legacy class supported attitude-rate commands and Phase 6 mission code may
  rely on them. Dropping `send_velocity_command_with_time` silently removes a
  timed-maneuver primitive. Both omissions violate G-5 (preserve all legacy
  control behaviors).
- Recommendation:
  Add `send_attitude(const mavros_msgs::msg::AttitudeTarget & cmd)` to
  `MavrosCommander` and document it in the triage table. Evaluate whether
  `send_velocity_timed(const Eigen::Vector4f & vel, double seconds)` belongs
  in `MavrosCommander` or `PosControl`; either way, include it and add a
  corresponding unit test.

### R-003 `CascadeController uses a single dt but the legacy cascade uses two separate timesteps`

- Severity: HIGH
- Section: `Spec / API Surface / cascade_controller.hpp / Implement Step 4`
- Type: Correctness
- Problem:
  In the legacy code there are two separate timestep fields:
  - `dt = 0.1` — used for the outer position-to-velocity PIDs and the yaw PID
  - `dt_pid_p_v = 1` — used for the inner velocity-to-acceleration PIDs

  In `input_pos_vel_xyz_yaw()` (PosControl.cpp line 371-385) and
  `input_pos_vel_1_xyz_yaw()` (lines 341-360), the velocity PIDs are called
  with `dt_pid_p_v`, not `dt`. This means the inner PID integrator accumulates
  at 1/1 s intervals while the outer loops at 1/0.1 s. The tuned gains
  (`POSCONTROL_VEL_XY_I = 0.40`, `POSCONTROL_VEL_XY_IMAX = 1000`) were set
  against this specific timestep ratio.

  The plan's `CascadeController::compute()` signature takes a single `dt`
  parameter and the step description says "Two-stage PID: position PIDs output
  velocity targets → velocity PIDs output accel". If both stages use the same
  `dt`, the inner-loop integral term will behave differently from legacy,
  breaking the existing gain tuning without any warning.
- Why it matters:
  This is a silent behavior change. The code will compile and run but the
  cascade controller will respond differently to position errors. On a real
  drone this could mean oscillation or sluggish response in acceleration mode.
  G-2 and G-5 are both violated.
- Recommendation:
  Add a second timestep field to `CascadeController::Config`:
  ```cpp
  struct Config {
    PID::Defaults pid_px, pid_py, pid_pz;
    PID::Defaults pid_vx, pid_vy, pid_vz;
    Limits limits;
    float dt_outer = 0.1f;   // position → velocity loop
    float dt_inner = 1.0f;   // velocity → acceleration loop
  };
  ```
  and update `compute()` accordingly. Document both fields in the YAML schema.

### R-004 `TrajectoryController couples back into VelocityController, preventing isolated testing`

- Severity: HIGH
- Section: `Spec / Invariants C-1 / API Surface / trajectory_controller.hpp`
- Type: Design
- Problem:
  All three public methods on `TrajectoryController` take a mutable
  `VelocityController &` parameter:
  ```cpp
  bool setpoint_world(..., VelocityController & vel_ctrl, ...);
  bool circle(..., VelocityController & vel_ctrl, ...);
  bool generator_world(..., VelocityController & vel_ctrl, ...);
  ```
  This design means:
  (a) `TrajectoryController` cannot be unit-tested without constructing a
  fully configured `VelocityController`.
  (b) It exposes the mutable internal state of `vel_ctrl` to the trajectory
  layer, which can silently alter PID integrators, limits, or reset state.
  (c) It creates a horizontal dependency between two sibling controllers,
  which makes the `PosControl` facade's composition semantics ambiguous.

  The trajectory methods in the legacy code call `input_pos_xyz_yaw()` which
  is a private method on `PosControl` itself — the tight coupling was
  intentional in the monolith. The refactoring should break that coupling by
  having trajectory methods return a target waypoint (not call a controller)
  and let `PosControl` pass the waypoint to `VelocityController`.
- Why it matters:
  Constraint C-1 states that control computation modules must be testable
  without ROS. If `TrajectoryController` requires a live `VelocityController`,
  the test for V-UT-8 and V-UT-9 becomes a two-class integration test, not a
  unit test. The design also violates the single-direction dependency graph
  in the ROADMAP.
- Recommendation:
  Remove `VelocityController &` from all `TrajectoryController` method
  signatures. Instead, have each method return the next waypoint:
  ```cpp
  // Returns {waypoint, done} — PosControl calls VelocityController with it
  std::pair<Eigen::Vector4f, bool> setpoint_world(
    const Eigen::Vector3f & pos_current,
    const Eigen::Vector4f & pos_target,
    float yaw_current, float dt);
  ```
  `PosControl` then chains: `traj_ctrl_.setpoint_world(...)` →
  `vel_ctrl_.compute(waypoint, ...)` → `commander_.send_velocity(...)`.

### R-005 `No replacement design for static-local state eliminated by Invariant I-6`

- Severity: HIGH
- Section: `Spec / Invariants I-6 / Implement Steps 3-5`
- Type: Correctness
- Problem:
  Invariant I-6 prohibits static local variables. The legacy code uses
  `static bool first` (and companion `static` accumulators) in at least 7
  methods to persist state across sequential calls to the same function:

  - `local_setpoint_command()` — first-call flag + one-shot publish
  - `send_velocity_command_with_time()` — start timestamp
  - `publish_setpoint_world()` — first-call flag + `pos_start`
  - `trajectory_setpoint()` — first-call flag + `pos_target_temp`
  - `trajectory_setpoint_world()` (both overloads) — first-call flag + target cache
  - `trajectory_circle()` — angle accumulator `t`
  - `trajectory_generator_world()` — first-call flag + position cache + count

  The plan acknowledges I-6 but provides no concrete design for what replaces
  these statics. Simply moving `static` to member variables is not sufficient
  — many of these flags are reset-on-arrival (true only once per new
  invocation sequence), which is an implicit state machine. Without a design,
  implementors will either silently re-add statics or produce incorrect
  behavior on the second call to the same high-level method.
- Why it matters:
  This is the largest behavioral correctness risk in the plan. The
  `trajectory_circle()` angle accumulator `t` in particular: if it becomes a
  member variable with no reset mechanism, calling `circle()` twice will start
  the second circle at the ending angle of the first, which changes the
  trajectory. The `trajectory_setpoint_world()` target cache prevents the PID
  from re-initializing on a new goal; losing that logic breaks re-entrant use.
- Recommendation:
  Round 01 must replace static state with an explicit per-method state struct
  or sub-state-machine. For trajectory methods:
  ```cpp
  // In TrajectoryController
  struct CircleState { float angle = 0.0f; bool active = false; };
  struct SetpointState { Eigen::Vector4f cached_target; bool initialized = false; };

  void reset_circle();    // called by PosControl before starting a new circle
  void reset_setpoint();  // called by PosControl before a new goal
  ```
  Each reset method must be called explicitly by `PosControl` when a new
  goal is issued, and the plan must document this reset protocol.

### R-006 `The triage table omits input_pos_xyz_yaw_without_vel and trajectory_setpoint (relative-frame variant)`

- Severity: MEDIUM
- Section: `Spec / Data Structure / File Migration Triage`
- Type: Completeness
- Problem:
  Two legacy methods are not listed in the triage table and are not mapped to
  any target:

  1. `input_pos_xyz_yaw_without_vel()` (PosControl.h line 353, PosControl.cpp
     lines 305-333) — a variant of the simple PID path that does not use
     `InertialNav` velocity feedback. It contains its own accumulating
     `static float yaw_diff_last` accumulator and is used in a commented-out
     call in `trajectory_setpoint_world()`. If it is intentionally dropped,
     that must be stated explicitly.

  2. `trajectory_setpoint()` (PosControl.h line 254, PosControl.cpp lines
     437-487) — the relative-frame variant of the trajectory method
     (`_pos_target = pos_now + pos_target`). This is distinct from
     `trajectory_setpoint_world()` which treats the target as absolute. The
     triage table maps only `trajectory_setpoint_world()` to
     `TrajectoryController::setpoint_world()`.
- Why it matters:
  Missing items in the triage table mean implementors have no guidance on
  whether these behaviors should be preserved, dropped, or merged into another
  method. G-5 (preserve all legacy control behaviors) cannot be verified
  against an incomplete triage.
- Recommendation:
  Add both methods to the triage table with an explicit action:
  - `input_pos_xyz_yaw_without_vel` — Drop if the velocity-feedback path is
    always preferred; or preserve as `VelocityController::compute_no_feedback()`.
  - `trajectory_setpoint` — Merge into `TrajectoryController::setpoint_relative()`
    with a documented coordinate-frame parameter, or drop with justification.

### R-007 `CMakeLists.txt target placement for control code is unspecified`

- Severity: MEDIUM
- Section: `Implement Step 9`
- Type: Spec Alignment
- Problem:
  The current `CMakeLists.txt` has three separate library targets:
  `drone_utils`, `drone_perception`, and `drone_drivers`. Step 9 says
  "Add new source files to `drone` target — no new library target needed",
  but there is no target named `drone` in the existing build system. The
  plan is referring to an assumed future target that does not yet exist.

  The control sources depend on `drone_utils` (for `drone::PID`,
  `drone::FuzzyPID`, `drone::TrajectoryGenerator`) and `drone_drivers` (for
  `drone::InertialNav`). `MavrosCommander` also depends on `rclcpp`,
  `mavros_msgs`, and `geometry_msgs`. If the control code is added to
  `drone_utils`, it drags ROS dependencies into a currently ROS-free utility
  library, violating Invariant I-1.
- Why it matters:
  Without a concrete target plan, the implementor either pollutes `drone_utils`
  with ROS headers or creates an ad-hoc target that is inconsistent with the
  existing CMake structure. The unit test linking (Step 10) also depends on
  the target name.
- Recommendation:
  Add a `drone_control` library target to the CMakeLists.txt plan:
  - Non-ROS sources (`VelocityController`, `CascadeController`,
    `TrajectoryController`, `yaw_utils`, `limits`) → link against `drone_utils`
  - ROS-coupled sources (`MavrosCommander`, `PosControl` facade) → separate
    target or a subsequent `drone_control_ros` layer
  - Unit tests link against `drone_control` (no ROS), integration tests link
    against the full stack.

### R-008 `Validation tests V-IT-2 and V-UT-12 are not self-contained without real hardware or a simulator`

- Severity: LOW
- Section: `Validation / Integration Tests`
- Type: Validation
- Problem:
  `V-IT-2: PosControl::go_to_position converges in simulation loop` requires
  either a live MAVROS node or a mock of `InertialNav` and `MavrosCommander`.
  The plan does not specify which approach is taken or how the simulation loop
  is driven. `V-UT-12: compute_fuzzy adapts gains based on error` tests fuzzy
  gain adaptation but the fuzzy system depends on the rule table loaded from
  YAML — the test requires a test fixture YAML or an inline config, which is
  not mentioned.
- Why it matters:
  If V-IT-2 requires a live MAVROS connection, it cannot run in the CI
  environment specified in the ROADMAP Phase 8 (Ubuntu 22.04 GitHub Actions).
  An untestable validation item does not contribute to the 80%+ coverage goal.
- Recommendation:
  For V-IT-2, specify a mock `InertialNav` that accepts injected position
  updates, and a mock `MavrosCommander` that captures published messages.
  For V-UT-12, provide an inline `FuzzyPID::Params` fixture so the test has
  no YAML file dependency.

---

## Trade-off Advice

### TR-1 `Prefer waypoint-returning trajectory API over controller-passing API`

- Related Plan Item: `API Surface / trajectory_controller.hpp`
- Topic: Testability vs Convenience
- Reviewer Position: Strongly prefer waypoint-return API
- Advice:
  Remove `VelocityController &` from `TrajectoryController` method signatures.
  Have methods return `std::pair<Eigen::Vector4f, bool>` (waypoint, done).
  `PosControl` composes the chain.
- Rationale:
  The waypoint-return API is independently testable: you can verify S-curve
  progression and circular path shape without constructing any PID object.
  The controller-passing API makes the trajectory module non-atomic and
  couples two sibling layers.
- Required Action:
  Adopt the waypoint-return API in round 01 or provide a detailed justification
  for why the coupling is acceptable.

### TR-2 `Consider a dedicated drone_control CMake target`

- Related Plan Item: `Implement Step 9`
- Topic: Build Clarity vs Simplicity
- Reviewer Position: Prefer dedicated target
- Advice:
  Introduce `drone_control` (non-ROS, links `drone_utils`) and optionally a
  thin `drone_control_ros` (links `drone_control + drone_drivers + rclcpp`)
  rather than putting everything into a single `drone` umbrella target.
- Rationale:
  The invariant I-1 (no ROS in computation modules) is easier to enforce and
  verify when the non-ROS and ROS layers are separate CMake targets. Tests for
  `VelocityController`, `CascadeController`, and `TrajectoryController` link
  only `drone_control`, which has no `rclcpp` dependency.
- Required Action:
  Round 01 must specify a concrete CMake target layout consistent with the
  existing `drone_utils` / `drone_drivers` pattern.

---

## Positive Notes

- The decision to isolate all MAVROS publishing into `MavrosCommander` is the
  correct architectural move and makes the invariant I-1 enforceable.
- Extracting the fuzzy rule base and PID gains to YAML is the right approach;
  the YAML structure described in the plan is sensible.
- Deferring `auto_tune` to Phase 8 is a well-reasoned trade-off (T-4). The
  legacy auto-tune uses `static` state throughout and depends on real hardware
  timing; forcing it into Phase 5 would introduce scope creep.
- The `yaw_utils` extraction is clean and the two test cases (V-UT-1, V-UT-2)
  cover the edge cases that matter.
- The decision to use a single cascade `compute()` facade rather than
  exposing `compute_position_stage()` / `compute_velocity_stage()` separately
  is correct — callers do not need to know about the internal staging.

---

## Approval Conditions

### Must Fix

- R-001 — FuzzyPID ownership model and 8-controller YAML schema
- R-002 — Add AttitudeTarget publisher and timed-velocity method
- R-003 — Separate outer and inner loop timesteps in CascadeController
- R-004 — Remove VelocityController coupling from TrajectoryController
- R-005 — Provide concrete state replacement design for all static-local state

### Should Improve

- R-006 — Complete the triage table (two missing legacy methods)
- R-007 — Specify concrete CMake target layout

### Trade-off Responses Required

- TR-1
- TR-2

### Ready for Implementation

- No
- Reason: The static-state replacement design (R-005) and the trajectory
  decoupling (R-004) are fundamental to the correctness of every method in
  `TrajectoryController`. The cascade timestep issue (R-003) is a silent
  behavior change that would break the tuned gains on real hardware.
  These three issues alone are sufficient to block implementation.
