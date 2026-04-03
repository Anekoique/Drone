# `pos-control` REVIEW `01`

> Status: Open
> Feature: `pos-control`
> Iteration: `01`
> Owner: Reviewer
> Target Plan: `01_PLAN.md`
> Previous Review: `00_REVIEW.md`
> Review Scope:
>
> - Verification of Round 00 blocker fixes (R-001 through R-008)
> - New issue discovery

---

## Verdict

- Decision: Approved for Implementation
- Blocking Issues: `0`
- Non-Blocking Issues: `3`

## Summary

All five HIGH blockers from Round 00 are genuinely fixed. The plan is now
structurally sound and the API surface is complete enough to implement without
ambiguity. Three residual issues are documented below — none of them are
blockers, but two are worth resolving during implementation to avoid surprises.

The key improvements from Round 01:
- `FuzzyConfig` ownership model correctly solves the dangling-pointer hazard
- `send_attitude()` and `send_velocity_timed()` are present in `MavrosCommander`
- Separate `dt_outer`/`dt_inner` in `CascadeController::Config` match legacy behavior
- `TrajectoryController` returns `{waypoint, done}` pairs with no `VelocityController` coupling
- Per-method state structs replace all static locals, and `new_goal()` documents the reset protocol
- Triage table is complete: `input_pos_xyz_yaw_without_vel` is explicitly dropped (NG-5), `trajectory_setpoint` maps to `setpoint_relative()`
- CMake target layout is concrete and matches the existing `drone_utils`/`drone_drivers` pattern

---

## Round 00 Blocker Verification

### R-001: FuzzyPID Ownership Model — VERIFIED FIXED

The `FuzzyConfig` struct (plan lines 163-180) owns the backing arrays via
`std::vector<float> mf_params` and `std::vector<std::array<float, kQfDefault>> rules`.
The `to_params()` method returns a `FuzzyPID::Params` whose raw pointer fields
point into those vectors, valid while `FuzzyConfig` lives. `FuzzyConfig` is a
member of `PosControl`, so it outlives all `FuzzyPID::init()` calls. This is
the ownership model recommended in R-001 option (a).

The YAML schema now lists all 8 controller parameter sets with distinct
`max_error` and `max_delta_error` values, matching the legacy constructor
(PosControl.h lines 136-145). The ordering (x, y, z, yaw, vx, vy, vz,
generic) aligns with the legacy `Fuzzy_params[8]` array structure.

`FuzzyPID::init()` (fuzzy_pid.cpp lines 129-169) is confirmed to copy all
raw-pointer data into `std::vector` members immediately — `mf_params_` via
`assign(p.mf_params, p.mf_params + 4 * kQfDefault)` (line 157) and
`rule_base_` row-by-row (lines 159-164). The pointer validity window is
correctly bounded to the duration of `init()`.

The plan inherits the legacy model of a single `FuzzyPID` object shared across
all 8 controller slots. This is architecturally unchanged from legacy and is
acceptable for this phase.

### R-002: MavrosCommander Completeness — VERIFIED FIXED

`send_attitude()` is present in the API surface (plan line 380) with the
corresponding `rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>` member
(line 396). The triage table now maps `setpoint_raw_attitude_publisher_` to
`MavrosCommander::send_attitude()` (plan line 142).

`send_velocity_timed()` is present (plan line 385) with member-level state
`timed_start_` and `timed_active_` replacing the static locals in the legacy
`send_velocity_command_with_time()` (PosControl.cpp lines 167-186).

Both publisher topic names are confirmed in Step 5 (plan lines 580-589),
matching the legacy constructor topics.

### R-003: CascadeController Dual Timestep — VERIFIED FIXED

`CascadeController::Config` (plan lines 274-279) has both `dt_outer = 0.1f`
and `dt_inner = 1.0f`, matching the legacy `dt = 0.1` and `dt_pid_p_v = 1`
fields. The compute method uses both stages. The default values preserve the
legacy gain-scaling ratio that the tuned PIDs depend on.

### R-004: TrajectoryController Decoupling — VERIFIED FIXED

All four `TrajectoryController` methods now return `std::pair<Eigen::Vector4f, bool>`.
No `VelocityController &` parameter appears anywhere in the API. The trajectory
path (plan lines 519-526) shows the correct chaining:
`traj_ctrl_.setpoint_world()` → waypoint → `vel_ctrl_.compute()` → commander.

### R-005: Static Local Replacement — VERIFIED FIXED

State structs are defined: `CircleState`, `SetpointState`, `GeneratorState`
(plan lines 308-323) for `TrajectoryController`; `GoToState`, `HoldState`
(plan lines 407-414) for `PosControl`. All are member variables, not static
locals. Reset methods `reset_setpoint()`, `reset_circle()`, `reset_generator()`,
`reset_all()` are declared. `new_goal()` calls `traj_ctrl_.reset_all()` and
zeroes the facade state structs. Constraint C-8 documents the caller obligation.

---

## New Findings

### N-001 `FuzzyConfig::rules stores std::array<float, kQfDefault> but rule_base needs 21 rows of 7 columns`

- Severity: HIGH
- Section: `Spec / Data Structure / FuzzyPID Ownership Model`
- Type: Correctness
- Problem:
  The plan defines:
  ```cpp
  std::vector<std::array<float, kQfDefault>> rules;   // 21 rows of 7
  ```
  `kQfDefault = kQfMiddle = 7` (fuzzy_pid.hpp line 16). So `rules` is a
  vector of 21 arrays of 7 floats — this part is consistent.

  However, `FuzzyPID::init()` expects `const float (*rule_base)[kQfDefault]` —
  a pointer to rows of 7 floats. The `to_params()` method must satisfy this
  pointer. If implemented as `rules.data()` after a contiguous
  `std::array<float, 7>` layout, the pointer arithmetic works because
  `std::vector<std::array<float, 7>>` is guaranteed contiguous in memory.

  The plan does not show the body of `to_params()`. There is one correctness
  trap: if the implementor casts `rules.data()` to `const float (*)[kQfDefault]`,
  that is well-defined only if `std::array<float, N>` has no padding, which is
  guaranteed by the standard. This is safe.

  The actual risk is that `FuzzyPID::init()` reads `fuzzy_output_num_ * kQfDefault`
  rows from `rule_base` (init line 160), where `fuzzy_output_num_` is 3 (for
  kp/ki/kd). The rule table has 21 rows = 3 * 7, which is correct. But
  `to_params()` must be called once for each controller slot 0-7 (all sharing
  the same rule table and mf_params), not once per slot with different rule data.
  The `FuzzyConfig` stores a single shared rule table, consistent with legacy.

  The plan's `to_params(int controller_id)` signature suggests it returns
  per-controller `Params`. The `Params` struct has `max_error` and
  `max_delta_error` as scalar fields — but `FuzzyPID::init()` reads an *array*
  of `Params` (indexed by `control_id`), storing `error_max_[i]` for each
  controller slot separately (init lines 138-143). Therefore `PosControl` must
  call `FuzzyPID::init()` with a `Params[8]` array, not with a single `Params`.

  The plan's `to_params(int controller_id)` returning a single `Params` is
  insufficient — `init()` needs a `const Params *` pointing to all 8 entries.
  The correct method signature should be:
  ```cpp
  std::array<FuzzyPID::Params, 8> to_params_array() const;
  ```
  or the caller must build the array manually and pass its `.data()` to `init()`.
  Without this correction, the implementor either calls `init()` with a
  single-element array (accessing index 1-7 out-of-bounds in init line 141) or
  builds an implicit array on the stack (fine, but not what `to_params()` suggests).
- Why it matters:
  `FuzzyPID::init()` iterates `for (int i = 0; i < p.control_id_count; ++i)`
  (line 140) reading `fuzzy_params[i]` — an out-of-bounds read on a one-element
  array is undefined behavior and will produce silent garbage for `error_max_[1]`
  through `error_max_[7]`.
- Fix:
  Rename `to_params(int controller_id)` to `to_params_array() const` returning
  `std::array<FuzzyPID::Params, 8>`. The caller invokes
  `fuzzy_pid_.init(fuzzy_config_.to_params_array().data())`. The method
  populates all 8 `Params` entries with the shared `mf_params` and `rule_base`
  pointers and per-controller `max_error`/`max_delta_error` values.

### N-002 `trajectory_setpoint_world second overload (with PID::Defaults) has no migration target`

- Severity: MEDIUM
- Section: `Spec / Data Structure / File Migration Triage`
- Type: Completeness
- Problem:
  The legacy `PosControl.h` (line 256) declares a second overload of
  `trajectory_setpoint_world()`:
  ```cpp
  bool trajectory_setpoint_world(
      Vector4f pos_now, Vector4f pos_target,
      PID::Defaults defaults, double accuracy, double yaw_accuracy,
      bool calculate_or_get_vel = false, float vel_x = DEFAULT_VELOCITY,
      float vel_y = DEFAULT_VELOCITY);
  ```
  This overload accepts a `PID::Defaults` parameter and temporarily overrides
  the x/y PID gains for the duration of the trajectory (PosControl.cpp lines
  526-570, `set_pid(pid_x, defaults)` called on entry and each iteration). The
  triage table maps only one `trajectory_setpoint_world()` to
  `TrajectoryController::setpoint_world()`. The overload with `PID::Defaults`
  is not listed.

  Whether a mission-layer caller uses this overload is not clear from the
  available context, but the absence from the triage table means it will
  silently be lost — no explicit "Drop" decision was recorded.
- Why it matters:
  G-5 (preserve all legacy control behaviors) cannot be confirmed without
  explicit triage of this overload. If Phase 6 mission code uses it to apply
  approach-specific PID tuning (e.g., a tighter gain during precision landing),
  the refactored API provides no equivalent path.
- Fix:
  Add an entry to the triage table: either Drop with justification (no known
  call site using the `defaults` overload) or map to a `setpoint_world_with_gains()`
  variant on `TrajectoryController` or to a `PosControl::trajectory_to_with_gains()`
  facade method. If dropped, document why in the non-goals section.

### N-003 `send_velocity_timed state is not reset on new call — stale timed_active_ risk`

- Severity: LOW
- Section: `Spec / API Surface / mavros_commander.hpp`
- Type: Correctness
- Problem:
  The plan replaces the static locals in `send_velocity_command_with_time()`
  with member fields `timed_start_` and `timed_active_`. In the legacy
  implementation (PosControl.cpp lines 165-186), `first = true` is reset when
  the timed command completes, which also re-arms the start timestamp on the
  next call. The member-state equivalent must reset `timed_active_ = false` on
  completion so a subsequent call to `send_velocity_timed()` starts a fresh
  timer.

  The plan documents the member fields but does not explicitly state the reset
  protocol for `send_velocity_timed()` the way it documents `new_goal()` for
  trajectory state. An implementor who follows the legacy pattern will get this
  right, but the spec is silent on it.
- Why it matters:
  If `timed_active_` is never reset to `false` after completion, a second call
  to `send_velocity_timed()` will immediately return `true` (already done) on
  the first invocation, skipping the timed maneuver entirely. This is the same
  category of bug as the static-local problem in R-005.
- Fix:
  Add one sentence to the `send_velocity_timed()` spec:
  "Returns `true` and resets `timed_active_` when duration has elapsed. A
  subsequent call begins a new timed command." No API change needed.

---

## Positive Notes

- The waypoint-return API for `TrajectoryController` is cleanly specified with
  all four methods returning `std::pair<Eigen::Vector4f, bool>`. The test cases
  V-UT-8, V-UT-9, V-UT-14, V-UT-15 exercise the trajectory methods in true
  isolation, confirming C-1 is enforceable.
- The CMake target layout (plan lines 98-111) is concrete, matches the existing
  `drone_utils`/`drone_drivers` pattern, and correctly splits the non-ROS
  `drone_control` sources from the ROS-coupled `mavros_commander.cpp` and
  `pos_control.cpp`. The `install()` note for `drone_control` is present.
- The `GeneratorState` struct captures the `TrajectoryGenerator` initialization
  pattern (`initialized` flag + `count`) without exposing the underlying
  generator state machine to the caller. This is the right level of abstraction.
- Dropping `publish_statistic()` is correctly justified: PAL statistics are
  not used in competition and have an optional compile-time dependency.
- The `new_goal()` reset protocol is clear and matches the invariant C-8. Phase 6
  mission code has unambiguous guidance on when to call it.

---

## Approval Conditions

### Must Fix Before Implementation

- N-001 — `to_params()` signature must produce a `Params[8]` array to avoid
  out-of-bounds read in `FuzzyPID::init()`. This is a HIGH correctness issue
  that will produce silent UB at runtime.

### Should Fix During Implementation

- N-002 — Add explicit triage row for the `PID::Defaults` overload of
  `trajectory_setpoint_world()`. Drop or preserve with justification.
- N-003 — Document the `timed_active_` reset in the `send_velocity_timed()` spec
  to prevent the silent skip-on-second-call bug.

### Ready for Implementation

- Yes, with the N-001 fix applied before writing `FuzzyConfig::to_params()`.
  The remaining two items (N-002, N-003) are low-risk and can be resolved
  inline during implementation without blocking progress.

---

## Review Summary

| Severity | Count | Status |
|----------|-------|--------|
| CRITICAL | 0     | pass   |
| HIGH     | 1     | warn   |
| MEDIUM   | 1     | info   |
| LOW      | 1     | note   |

Verdict: WARNING — 1 HIGH issue (N-001) must be resolved before implementing
`FuzzyConfig::to_params()`. All Round 00 blockers are genuinely fixed. The plan
is otherwise ready for implementation.
