# `mission` REVIEW `00`

> Status: Open
> Feature: `mission`
> Iteration: `00`
> Owner: Reviewer
> Target Plan: `00_PLAN.md`
> Previous Review: N/A (first round)
> Review Scope:
>
> - Plan correctness against legacy OffboardControl behavior
> - Spec alignment with ROADMAP Phase 6
> - Design soundness (handler pattern, dependency flow)
> - API completeness for DroneNode mission dispatch
> - Migration completeness (triage table vs. legacy methods/members)
> - Dependency analysis (NodeBase, Motors, InertialNav integration)
> - State machine flow against legacy sequence
> - Frame transform coverage

---

## Verdict

- Decision: Blocked — Do Not Implement
- Blocking Issues: `7`
- Non-Blocking Issues: `4`

## Summary

The plan captures the broad decomposition correctly (four handlers, frame
transforms module, MissionConfig, DroneNode dispatch) and the invariants are
sound. However, reading the legacy StateMachine.cpp and OffboardControl.h/cpp
in full reveals seven issues that will cause either incorrect flight behavior,
API compilation failures, or mission-critical state loss at runtime. None of
them are stylistic — all seven involve missing behavior that exists in the
legacy code and has no equivalent in the proposed plan.

The most serious issues are: (1) the `Doshot` state is mapped to
`AirdropHandler::execute()` but the actual legacy `Doshot()` is only one
sub-function called from inside the much larger `FlyState::Doshot` handler in
StateMachine.cpp — the outer state machine logic (world-position approach,
clustering-driven waypoint override, `doshot_scout` patrol fallback,
`fast_mode_` path) is unaccounted for; (2) `LandToStart` is a live state in
the legacy enum and has a full handler in StateMachine.cpp but it appears
nowhere in the new `FlyState` enum or triage table; (3) the two
`rotate_realglobal2stand` / `rotate_realstand2global` transform functions are
missing from `frame_transforms.hpp` despite being distinct from the compass
transforms; (4) the `Doland` sub-state machine is more complex than the plan's
`LandingHandler` sub-states — the plan omits the visual landing approach
(`catch_target` on `TARGET_TYPE::H`) and the spiral search pattern (`surround_land`);
(5) `MissionConfig` is missing five fields that `read_configs()` reads from
YAML; (6) `DroneNode` inherits `rclcpp::Node` directly but the project
standard for the node base is `NodeBase` (from Phase 4), and the plan does not
address how `debug_mode_`, `fast_mode_`, and system-status readiness guards
in `timer_callback` carry over; (7) the `WaypointState` struct exposes a raw
`Timer` member without specifying ownership, conflicting with invariant I-3
(frame transforms stateless) and I-4 (references at construction).

---

## Blocking Issues

### B-001 `LandToStart` state is missing from FlyState enum and triage table

- Severity: HIGH
- Section: `Spec / Data Structure / File Migration Triage` and `API Surface`
- Type: Correctness / Completeness
- Problem:
  The legacy `FlyState` enum in `StateMachine.h` (line 34) includes
  `LandToStart`. `StateMachine.cpp` lines 392-437 implement a full handler for
  it with three sub-states (`land_to_start_init`, `land_to_start_wait`,
  `land_to_start_end`). The sequence: switch to GUIDED, fly world setpoint
  (0, 0, 2), wait 19 seconds or until STANDBY, then switch LAND and
  transition to `end`. It is used as an alternative landing path independent
  of `Doland`.

  The new plan's `FlyState` enum (plan lines 200-209) has no `land_to_start`
  variant. The triage table has no row for `LandToStart`. The
  `LandingHandler`'s `SubState` enum covers only `{rtl, wait, guided, land}`,
  which maps to the legacy `FlyState::Doland` handler — not `LandToStart`.

  The `FlyStateMap` string table in `StateMachine.h` (line 54) maps
  `"LAND_TO_START"` to `FlyState::LandToStart`, confirming it is a
  terminal-control-accessible state that operators can invoke manually.
- Why it matters:
  If the operator sends the `LAND_TO_START` command during a competition run,
  or if future code paths target it, the new DroneNode has no handler for it
  and will silently remain in whatever state it was in before. G-7 (preserve
  all legacy mission behavior) is violated.
- Fix:
  Add `land_to_start` to the `FlyState` enum. Add a row to the triage table
  mapping `FlyState::LandToStart` to a new `LandingHandler::land_to_start()`
  method or a merged sub-state within `LandingHandler`. Add a `land_to_start:`
  case to `DroneNode::timer_callback()`.

### B-002 `Doshot` outer handler logic is not captured — only the inner sub-function is

- Severity: HIGH
- Section: `Spec / Data Structure / File Migration Triage`
- Type: Correctness / Completeness
- Problem:
  The triage table maps `OffboardControl::Doshot()` to
  `AirdropHandler::execute()`. However `OffboardControl::Doshot()` (cpp lines
  367-680) is a _sub-function_ that handles only the final pixel-level
  approach for a single barrel — it is not the same as the
  `FlyState::Doshot` handler in StateMachine.cpp (lines 69-299).

  The `FlyState::Doshot` handler in StateMachine.cpp contains all of the
  following logic that is absent from the plan's `AirdropHandler::execute()`:

  - World-position approach: fly to `cal_center[counter].point` at
    `shot_halt_low` altitude using `send_world_setpoint_command()` while the
    drone is still more than `max_accurate` meters away or the timer is under
    6 seconds (lines 178-212).
  - `fast_mode_` fast drop path: immediately fire servo without visual
    approach when `fast_mode_` is true (lines 213-219).
  - `circle_counter` fallback: after 12 consecutive cycles with no CIRCLE
    detection, invoke `waypoint_goto_next()` to patrol the shot zone (lines
    220-226).
  - `doshot_wait` counter management: decides whether to re-attempt the
    second barrel (`shot_counter <= 1` check, lines 246-277).
  - `doshot_end` double-open-servo safety: opens servo 11 and 12 on entry
    before 2-second wait (lines 279-291).

  The plan's `AirdropHandler` sub-states are `{init, shot, wait, end}` with
  the description "fly toward using PosControl", but this collapses at least
  three distinct control strategies into an unspecified "fly toward."
- Why it matters:
  Implementing `AirdropHandler::execute()` from the plan spec alone will
  produce an airdrop handler that skips the pre-approach world-position
  phase, cannot handle `fast_mode_`, has no circle-loss fallback, and does
  not perform the second-barrel logic. The airdrop phase will fail in any
  real flight run.
- Fix:
  Expand the `AirdropHandler` sub-state machine in the plan to match the
  actual `FlyState::Doshot` handler. Required sub-states:
  `{init, world_approach, pixel_approach, wait, end}`. Document
  `fast_mode_` as a flag sourced from `NodeBase::fast_mode()`. Document the
  `circle_counter` fallback threshold (12 cycles). Explicitly triage
  `fast_mode_` into `MissionConfig` or `DroneNode`.

### B-003 Two frame transform functions missing from `frame_transforms.hpp`

- Severity: HIGH
- Section: `Spec / API Surface / frame_transforms.hpp`
- Type: Completeness
- Problem:
  The legacy `OffboardControl.h` declares six rotate functions (lines
  304-353):
  1. `rotate_global2stand()` — rotates by `-headingangle_compass`
  2. `rotate_stand2global()` — rotates by `+headingangle_compass`
  3. `rotate_realglobal2stand()` — rotates by `-headingangle_real`
  4. `rotate_realstand2global()` — rotates by `+headingangle_real`
  5. `rotate_world2start()` — rotates by `start.w()` (start yaw)
  6. `rotate_world2local()` — rotates by `get_world_yaw()` (current yaw)
  7. `rotate_local2world()` — rotates by `-get_world_yaw()`

  The plan's `frame_transforms.hpp` API (plan lines 220-237) provides:
  `rotate_xy`, `world_to_compass`, `compass_to_world`, `world_to_start`,
  `world_to_local`, `zone_origin_to_world` — six functions total.

  Functions 3 and 4 (`rotate_realglobal2stand` / `rotate_realstand2global`)
  using `headingangle_real` are not present. `headingangle_real` is a
  separate config field (`headingangle_real` in `OffboardControl.yaml`,
  loaded at line 610) that defaults to `headingangle_compass` when absent.
  It is used in `Doland()` (line 726: `rotate_global2stand(scout_x,
  scout_y + 0.3, x_home, y_home)`) — and the search reveals it is used
  separately in `autotune()` (line 903). Even if no active mission state
  currently invokes those two functions by their exact name, `MissionConfig`
  must carry `heading_real_rad` and the transforms must be available.

  Additionally, `rotate_local2world()` (rotate by `-get_world_yaw()`) is
  used in `Doshot()` (line 520) and `timer_callback()` (line 65) but
  does not appear as a named function in `frame_transforms.hpp`. The plan
  comments say `world_to_local` covers it, but `world_to_local` and
  `local_to_world` are inverses and must be separately named.

- Why it matters:
  The airdrop handler uses `rotate_local2world` (line 520 in
  `Doshot()`). Without an explicit `local_to_world()` function, the
  implementor must reconstruct the inverse manually, risking sign errors.
  If `heading_real_rad` is missing from transforms, the landing visual
  approach cannot correctly rotate the scout offset.
- Fix:
  Add `local_to_world(float x, float y, float current_yaw)` to the API.
  Add `compass_to_world_real(float x, float y, float heading_real_rad)` and
  `world_to_compass_real(float x, float y, float heading_real_rad)` (or
  document that `compass_to_world` with `heading_real_rad` is the correct
  call). Confirm `heading_real_rad` is populated in `MissionConfig`.

### B-004 `MissionConfig` is missing five fields from `read_configs()`

- Severity: HIGH
- Section: `Spec / Data Structure / MissionConfig`
- Type: Completeness
- Problem:
  `OffboardControl::read_configs()` (lines 606-644) reads the following
  fields from `OffboardControl.yaml`:

  | YAML key | Legacy member |
  |---|---|
  | `headingangle_compass` | `headingangle_compass` |
  | `headingangle_real` | `headingangle_real` |
  | `dx_shot` | `dx_shot` |
  | `dy_shot` | `dy_shot` |
  | `dx_see` | `dx_see` |
  | `dy_see` | `dy_see` |
  | `shot_halt` | `shot_halt` |
  | `shot_halt_surround` | `shot_halt_surround` |
  | `shot_halt_low` | `shot_halt_low` |
  | `see_halt` | `see_halt` |
  | `drone_to_camera_x/y/z` | `drone_to_camera[0..2]` |
  | `servo_open_position` | `servo_open_position` |
  | `servo_close_position` | `servo_close_position` |
  | `shot_big_target` | `shot_big_target` |

  The proposed `MissionConfig` struct (plan lines 161-189) covers:
  `heading_compass_deg`, `heading_real_rad`, `default_yaw`,
  `shot_zone.altitude`, `shot_zone.altitude_low`, `recon_zone.altitude`,
  `servo_open_pwm`, `servo_close_pwm`, `prefer_large_target`,
  `drone_to_camera`, `takeoff_altitude`, `waypoint_accuracy`.

  Missing from `MissionConfig`:
  1. `shot_zone.altitude_surround` — `shot_halt_surround` (3.0 m default,
     used in the `circle_counter` fallback patrol in StateMachine.cpp line
     226).
  2. `shot_zone.dx` / `shot_zone.dy` — `dx_shot`, `dy_shot` are zone
     _origin_ offsets in the compass frame, which are separate from
     `ZoneConfig.length`/`width`. The plan's `ZoneConfig.dx`/`dy` fields
     exist (plan line 164) but the `mission.yaml` already has `dx_shot` and
     `dy_shot` as separate keys, and the `MissionConfig::load()` must read
     them under the correct YAML key name. The mapping between the new field
     names and YAML keys is unspecified.
  3. `bucket_height` — `bucket_height = 0.3` (line 539) is used in
     `timer_callback` (line 99), `Doshot()` (line 480, 526), and
     `Doland()` (line 693). It is not present in `MissionConfig`.
  4. The `can_config.yaml` fields (`radius`, `accuracy`, `shot_duration`,
     `shot_wait`, `tar_z`, `shot_target_x_r/l`, etc.) are read inside
     `Doshot()` itself, not via `read_configs()`. The plan says
     `MissionConfig::load()` replaces `read_configs()` but these per-approach
     parameters are loaded from a different YAML file that is not mentioned
     in the plan at all.
  5. The `land_config.yaml` fields (`scout_halt`, `scout_x`, `scout_y`,
     `accuracy`, `tar_x`, `tar_y`, `tar_z`) used in `Doland()` are also
     not present in `MissionConfig` and the config file is not referenced.

- Why it matters:
  `MissionConfig::load()` will be implemented without knowledge of
  `can_config.yaml` and `land_config.yaml`. The airdrop and landing handlers
  will lack the per-phase tuning parameters they need. G-4 (MissionConfig
  consolidates all YAML config) cannot be satisfied with the current struct
  definition.
- Fix:
  Add `shot_zone.altitude_surround` and `bucket_height` to `MissionConfig`.
  Specify the YAML key names for `dx`/`dy` in `ZoneConfig`. Add sections
  `AirdropConfig` (from `can_config.yaml`) and `LandingConfig` (from
  `land_config.yaml`) to `MissionConfig`, or explicitly decide in the
  non-goals that these are deferred, citing the specific fields.

### B-005 `DroneNode` inherits `rclcpp::Node` directly, bypassing `NodeBase`

- Severity: HIGH
- Section: `Spec / API Surface / drone_node.hpp`
- Type: Design / Correctness
- Problem:
  The plan (line 367) declares:
  ```cpp
  class DroneNode : public rclcpp::Node {
  ```
  Phase 4 produced `NodeBase` (include/drone/drivers/node_base.hpp), which
  inherits `rclcpp::Node` and adds:
  - `debug_mode()`, `print_info()`, `sim_mode()`, `fast_mode()` accessors
    backed by ROS 2 declared parameters
  - `elapsed_time()` / `start_time()` / `set_start_time()` helpers
  - The `mode_client_` for mode switching

  The legacy `timer_callback()` uses `debug_mode_` in four separate guards
  (lines 36, 57, 72, 81), uses `print_info_` in one guard (line 36), and the
  `FlyState::Doshot` handler uses `fast_mode_` (line 213). The `FlyState::end`
  handler (StateMachine.cpp line 37) uses `get_start_time()` and
  `get_cur_time()`. These all flow from `OffboardControl_Base` (the legacy
  equivalent of `NodeBase`).

  If `DroneNode` inherits raw `rclcpp::Node`, none of these mode flags are
  present. The system-status readiness guard (lines 37-55 in
  `timer_callback()`) also does not appear in the plan's `timer_callback()`
  pseudocode — this guard is the first thing executed on every tick and
  prevents mission execution until MAVROS is ready and GPS data is valid.
- Why it matters:
  Without `NodeBase` inheritance: `fast_mode()` is unavailable to handlers
  that need it; the system-status readiness guard cannot be implemented as
  written; the `print_info()` / `debug_mode()` guards silently disappear.
  A `DroneNode` that starts dispatching mission state before GPS is valid
  will command the drone to move to uninitialized positions.
- Fix:
  Change `DroneNode` to inherit `NodeBase` instead of `rclcpp::Node`.
  Add the system-status readiness guard and the position data validity check
  at the top of `timer_callback()` before the state switch. Pass
  `NodeBase::fast_mode()` to `AirdropHandler` via `Subsystems` or a
  dedicated flag.

### B-006 `Doland` sub-state machine does not match the legacy sequence

- Severity: HIGH
- Section: `Spec / API Surface / landing_handler.hpp`
- Type: Correctness
- Problem:
  The plan describes `LandingHandler::SubState` as `{rtl, wait, guided,
  land}` with the comment "RTL -> wait -> GUIDED -> LAND" and references
  the legacy `Doland` method.

  The legacy `FlyState::Doland` handler in StateMachine.cpp (lines 341-389)
  does implement RTL -> wait 18s -> GUIDED -> call `Doland()`. However, the
  legacy `OffboardControl::Doland()` method (cpp lines 682-818) is the
  _visual landing approach_ that runs after entering GUIDED mode:

  1. Reads `land_config.yaml` for `scout_halt`, `scout_x`, `scout_y`, etc.
  2. Flies to `rotate_global2stand(scout_x, scout_y + 0.3)` at
     `scout_halt` altitude.
  3. In a loop, checks `is_get_target(TARGET_TYPE::H)`. If no H target for
     more than 2 seconds, executes a linear search: flies to
     `rotate_global2stand(scout_x + surround_land * 1.0, scout_y)` and
     increments `surround_land` (range -3 to +3).
  4. When H is detected, calls `catch_target(defaults, TARGET_TYPE::H, ...)`
     for pixel-level approach.
  5. After landing timeout (19 s) or arrival, sends
     `send_velocity_command_with_time(0, 0, -0.2, 0, 1)` — a 0.2 m/s
     downward velocity command for 1 second.
  6. Returns true; outer handler then switches LAND mode.

  None of steps 3-5 appear in the plan. `LandingHandler::execute()` has no
  detector interface, no search pattern, no pixel approach, and no velocity
  sink command.
- Why it matters:
  The landing handler as specified will RTL, wait, switch GUIDED, and
  immediately switch to LAND mode — it will not attempt precision landing
  over the H marker. The drone will land wherever the RTL brought it, not
  at the designated landing target.
- Fix:
  Add `LandingHandler::SubState::visual_approach` after `guided`. Document
  the `surround_land` search pattern (linear ±3m at 1m increments). Add a
  note that detector integration is deferred (Phase 7) but the sub-state
  must be present with a stub. Add
  `PosControl::send_velocity_timed()` invocation before LAND mode switch.

### B-007 `WaypointState` contains a `Timer` member, conflicting with stateless-free-function goal and causing ownership ambiguity

- Severity: HIGH
- Section: `Spec / API Surface / waypoint_nav.hpp`
- Type: Design
- Problem:
  The plan defines:
  ```cpp
  struct WaypointState {
    int index = 0;
    bool initialized = false;
    Timer timer;
  };
  ```
  `WaypointState` is a value type held as a member inside `AirdropHandler`
  (`wp_state_`) and `ReconHandler` (`wp_state_`). The `Timer` class (from
  Phase 1) almost certainly stores a `std::chrono::steady_clock::time_point`
  internally, making `WaypointState` non-trivially constructible.

  Two design problems arise:
  1. The free function `waypoint_goto_next()` takes `WaypointState & state`
     by reference and modifies it (advances `index`, resets `timer`). This
     makes the function stateful through the reference parameter — it is
     not stateless in the sense that matters for testing. This is
     acceptable, but it conflicts with invariant I-3 ("frame transforms are
     stateless free functions") if the reader interprets I-3 to apply to all
     `waypoint_nav.hpp` functions. The invariant must be clarified.
  2. `WaypointState::timer` is default-constructed at `DroneNode` construction
     time and starts counting immediately. The `Timer` semantics must be
     specified: does it start on construction or on first `reset()` call?
     The legacy `waypoint_timer_` is reset in `StateMachine::transition_to()`
     (cpp line 598) on every state transition. The plan has no equivalent
     reset trigger; the `WaypointState` passed to `goto_zone()` must be
     reset at the right moment or the first waypoint will be skipped.
- Why it matters:
  If `wp_state_.timer` starts at construction and `AirdropHandler::goto_zone()`
  is not called immediately, the timer will have accumulated time before
  the first use. The first waypoint timeout check may fire immediately,
  skipping directly to the next waypoint. This is the static-local reset
  category of bug.
- Fix:
  Add a `reset()` method to `WaypointState`. Document that `DroneNode` must
  call `airdrop_.reset()` and `recon_.reset()` on entering the respective
  states (i.e., when transitioning from `goto_shot` → `airdrop` and from
  `goto_recon` → `recon_patrol`). Clarify invariant I-3 to read "frame
  transforms in `frame_transforms.hpp` are stateless" and exclude
  `waypoint_nav.hpp`.

---

## Non-Blocking Issues

### N-001 `Goto_shotpoint` and `Goto_scoutpoint` use time-based arrival, not position accuracy — this must be documented

- Severity: MEDIUM
- Section: `Spec / Implement / Execution Flow`
- Type: Correctness
- Problem:
  `AirdropHandler::goto_zone()` and `ReconHandler::goto_zone()` are
  specified to "navigate to zone, returns `airdrop`/`recon_patrol` when
  arrived." The caller will naturally assume position-based arrival (within
  `waypoint_accuracy`).

  The legacy `FlyState::Goto_shotpoint` handler (StateMachine.cpp lines
  48-65) returns after `waypoint_timer_.elapsed() > 12` seconds — a fixed
  timeout, not a position check. Similarly `FlyState::Goto_scoutpoint`
  (lines 302-320) transitions after `waypoint_timer_.elapsed() > 7.5`
  seconds. Neither checks arrival position.

  The entry point altitude also matters: `Goto_shotpoint` uses
  `shot_halt` (not `shot_halt_surround` or `shot_halt_low`), and
  `Goto_scoutpoint` uses `see_halt`. These specific altitudes must appear in
  the `goto_zone()` API or its documentation.
- Why it matters:
  If the implementor writes position-based arrival, the timeout behavior
  changes and timing-critical competition runs may fail. Without specifying
  the altitude used, the implementor may use the wrong `ZoneConfig` field.
- Fix:
  Add a `timeout_sec` parameter to `goto_zone()` signatures with defaults
  matching legacy (12.0s for shot, 7.5s for scout). Document which altitude
  field is used.

### N-002 `fly_state_to_int()` mapping is not in the plan but is needed for the state publisher

- Severity: MEDIUM
- Section: `Spec / API Surface / drone_node.hpp`
- Type: Completeness
- Problem:
  The legacy `publish_current_state()` (OffboardControl.cpp line 279)
  calls `fly_state_to_int(state)` to publish an integer encoding on
  `"current_state"` topic. The mapping encodes mission semantics (e.g.,
  `Doshot` publishes 0, `Surround_see` publishes 3, `Doland` publishes 4)
  that other system components may consume.

  The plan publishes state on `/drone/state` via a
  `Publisher<std_msgs::msg::Int32>` but does not specify the encoding. If
  the new `FlyState` enum ordinals change (they will — `finished` is not in
  legacy, `goto_shot` replaces `Goto_shotpoint`, etc.), any consumer of
  `current_state` topic that used the legacy integer values will break.
- Why it matters:
  The state integer encoding is part of the external interface.
  Ground-station tools or cv_py components may consume it.
- Fix:
  Define an explicit `fly_state_to_int()` function in `mission_types.hpp`
  with the same integer mapping as legacy (or document that the encoding has
  changed). Add the topic name to the constraints or the DroneNode spec.

### N-003 `is_first_run_` sentinel reset pattern has no equivalent in the handler design

- Severity: LOW
- Section: `Spec / Implement / Airdrop Sub-state Flow` and `Spec / Constraints`
- Type: Correctness
- Problem:
  The legacy `FlyState::Doshot` handler (StateMachine.cpp line 71) and
  `FlyState::Doland` handler (line 349) both check `owner_->is_first_run_`
  to reset their sub-state machines on the first entry into the state.
  `StateMachine::transition_to()` sets `is_first_run_ = true` on every
  state transition (cpp line 605, confirmed by the reset logic).

  The plan's `AirdropHandler::reset()` and `LandingHandler::reset()`
  methods exist on the API, but the plan does not specify when `DroneNode`
  must call them. The `timer_callback()` pseudocode shows:
  ```
  airdrop:  state_ = airdrop_.execute(...)
  ```
  with no `reset()` call shown on state entry. If `DroneNode` never calls
  `reset()` before `execute()`, the handler sub-state begins in whatever
  state it was left from a previous run.
- Why it matters:
  On the first competition run this may work. In any scenario where the
  state machine cycles back (e.g., testing `Reflush_config` or manual
  state override), the airdrop handler will skip its `init` sub-state and
  attempt to shoot with stale servo counters.
- Fix:
  Add one sentence to constraint C-7 or add a constraint C-8: "DroneNode
  must call `handler.reset()` exactly once when entering the state for the
  first time." Show the `reset()` call in the `timer_callback()` pseudocode
  at state entry, not at state re-entry.

### N-004 ROADMAP Phase 6 lists `state_machine.hpp` as a target file; the plan omits it

- Severity: LOW
- Section: `Spec / Architecture`
- Type: Alignment
- Problem:
  `ROADMAP.md` (line 165) lists:
  ```
  state_machine | StateMachine.h/cpp | include/drone/mission/state_machine.hpp
  ```
  as a Phase 6 deliverable. The plan's architecture section lists no
  `state_machine.hpp` file — the enum dispatch is inlined into `drone_node.hpp`.
  The plan also uses `frame_transforms.hpp` and `waypoint_nav.hpp` which
  are not listed in the ROADMAP's Phase 6 table, though they are reasonable
  additions.

  This is a minor alignment gap: the ROADMAP expected a discrete
  `state_machine.hpp` artifact.
- Why it matters:
  A reviewer checking ROADMAP deliverables against plan outputs will find
  the `state_machine.hpp` entry unchecked. In a CI gate that validates
  deliverable files, this would fail.
- Fix:
  Either create a minimal `state_machine.hpp` that defines the `FlyState`
  enum and the dispatch interface (could be a thin wrapper around
  `mission_types.hpp`), or update the ROADMAP to note that the state machine
  is embedded in `DroneNode` and `mission_types.hpp` per the design
  trade-off T-1.

---

## Positive Notes

- The choice to use value-return `FlyState` from handlers instead of
  callback indirection is clean and directly testable. Trade-off T-1 is
  well-reasoned.
- Invariants I-1 through I-7 are precise and enforceable at the CMake
  target level. The single `drone_mission` library linking
  `drone_control + drone_drivers` correctly captures the dependency flow.
- `ZoneConfig` with normalized 0-1 waypoints is a good improvement over
  the legacy hardcoded absolute coordinate arrays. The `surround_shot_points`
  and `surround_see_points` arrays in the legacy code are candidates for
  config-driven replacement, and the plan correctly plans this.
- Deferring TensorRT detector integration to Phase 7 (Trade-off T-2) is
  correct. The stub detector interface in `AirdropHandler` is the right
  approach.
- Dropping `publish_statistics()` / `MYPID` / `Termial_Control` /
  `Reflush_config` with explicit non-goals (NG-4, NG-5) prevents scope
  creep. These are competition-day debug aids, not flight-critical code.
- Unit tests V-UT-1 through V-UT-3 on frame transform round-trips are
  well-scoped and will catch coordinate-system errors early.

---

## Approval Conditions

### Must Fix Before Implementation

- B-001 — Add `land_to_start` to `FlyState` enum and `LandingHandler` API.
- B-002 — Expand `AirdropHandler` sub-state machine to match the full
  `FlyState::Doshot` handler logic, including world-approach, fast-mode,
  circle-loss fallback, and double-barrel logic.
- B-003 — Add `local_to_world()` to `frame_transforms.hpp`. Clarify
  `heading_real_rad` transform coverage.
- B-004 — Add `shot_zone.altitude_surround`, `bucket_height`, and sections
  for `can_config.yaml` / `land_config.yaml` fields to `MissionConfig`.
- B-005 — Change `DroneNode` to inherit `NodeBase`. Add system-status
  readiness guard and GPS validity check to `timer_callback()` pseudocode.
- B-006 — Add visual landing approach sub-state, search spiral, and velocity
  sink command to `LandingHandler`.
- B-007 — Document `WaypointState::reset()` protocol and the state-entry
  reset trigger in `DroneNode`.

### Should Fix During Implementation

- N-001 — Document time-based arrival in `goto_zone()` signatures with
  explicit timeout defaults.
- N-002 — Define `fly_state_to_int()` mapping in `mission_types.hpp`.

### Can Defer

- N-003 — Clarify `reset()` call obligation in constraints.
- N-004 — Reconcile `state_machine.hpp` ROADMAP entry vs. plan architecture.

---

## Review Summary

| Severity | Count | Status |
|----------|-------|--------|
| CRITICAL | 0     | pass   |
| HIGH     | 7     | block  |
| MEDIUM   | 2     | warn   |
| LOW      | 2     | note   |

Verdict: BLOCKED — 7 HIGH issues must be resolved before implementation.
The airdrop handler (B-002), landing handler (B-006), and missing state
(B-001) represent the largest gaps: together they cover roughly 60% of the
actual competition flight behavior that is absent from the current plan.
