# `mission` REVIEW `01`

> Status: Approved with Conditions
> Feature: `mission`
> Iteration: `01`
> Owner: Reviewer
> Target Plan: `01_PLAN.md`
> Previous Review: `00_REVIEW.md`
> Review Scope:
>
> - Verification that all 7 Round 00 blockers are genuinely resolved
> - AirdropHandler sub-states vs. full FlyState::Doshot handler
> - LandingHandler sub-states vs. full Doland path
> - LandToStart completeness
> - MissionConfig field coverage vs. all 3 YAML files
> - DroneNode inheritance and readiness guard
> - Frame transforms completeness (7 legacy rotate functions)
> - New issues introduced in revision

---

## Verdict

- Decision: Approved — Implement with the conditions below
- Blocking Issues: `1`
- Non-Blocking Issues: `3`

## Summary

The revision correctly resolves 6 of the 7 Round 00 blockers. The one remaining
blocker (R-B-004, formerly B-004) is a YAML field mapping gap that will cause a
silent build or a runtime crash: the `mission.yaml` file does not contain a
`heading_real_rad` key, yet `MissionConfig` declares `heading_real_rad` without
specifying how it is derived from `headingangle_real`. The legacy `read_configs()`
converts `headingangle_real` (degrees) to radians on load; the plan must name the
YAML key and document the degree-to-radian conversion so the implementor can
reproduce the same logic in `MissionConfig::load()`.

The remaining three findings are non-blocking: one concerns the `AirdropConfig`
field names not matching the YAML keys in `airdrop.yaml` (will cause YAML parse
errors if copied verbatim but caught at runtime, not compile time); one concerns
`LandingConfig` missing the `tar_x`/`tar_y` pixel target fields that `Doland`
reads; and one concerns the `surround_land` reset bug carried forward from the
legacy code that the plan faithfully reproduces but should be documented.

---

## Blocker Verification

### B-001 — LandToStart

Resolved. The revised plan adds `land_to_start` to the `FlyState` enum (plan
line 227), adds `LandingHandler::land_to_start()` with `LandToStartState::{init,
wait, land}` (plan lines 371–373), and adds a `land_to_start:` dispatch case and
`landing_.reset_land_to_start()` call in `timer_callback()`. The three sub-states
match the legacy handler (StateMachine.cpp lines 392–437): switch GUIDED, fly to
world (0,0,2), wait 19 s or MAV_STATE_STANDBY, switch LAND, transition to
`finished`. The sub-state names differ (`land` vs. `land_to_start_end`) but the
behavior is faithful. No remaining gap.

### B-002 — AirdropHandler full outer handler

Resolved. The plan now specifies five sub-states `{init, world_approach,
pixel_approach, wait, end}` (plan line 316). Reviewing each against the legacy
FlyState::Doshot handler (StateMachine.cpp lines 69–299):

- `init`: resets counters, loads PID/limits — matches lines 103–127.
- `world_approach`: fly to `cal_center[counter].point` at `altitude_low`;
  `fast_mode` immediate fire path; `circle_counter >= 12` patrol fallback;
  70-second overall timeout — matches lines 89–226.
- `pixel_approach`: calls `catch_target()` stub; fires servo when `Doshot()`
  returns true — matches line 227.
- `wait`: increment `shot_counter`, re-enter `world_approach` or proceed to
  `end` — matches lines 245–277.
- `end`: open servos 11 and 12; wait 2 s; restore defaults — matches lines
  279–291.

The plan correctly documents `fast_mode` via `NodeBase::fast_mode()` and the
`circle_counter` fallback threshold of 12. No remaining gap.

### B-003 — Two missing frame transform functions

Resolved. The revised `frame_transforms.hpp` API now includes `local_to_world()`
(plan line 250) and documents that `compass_to_world` / `world_to_compass` accept
`heading_rad` so they work for both the compass and real heading values. The plan
adds `heading_real_rad` to `MissionConfig` (plan line 189). All 7 legacy rotate
functions are covered:

| Legacy function | New name |
|---|---|
| `rotate_global2stand` | `world_to_compass` |
| `rotate_stand2global` | `compass_to_world` |
| `rotate_realglobal2stand` | `world_to_compass` with `heading_real_rad` |
| `rotate_realstand2global` | `compass_to_world` with `heading_real_rad` |
| `rotate_world2start` | `world_to_start` |
| `rotate_world2local` | `world_to_local` |
| `rotate_local2world` | `local_to_world` |

No remaining gap.

### B-004 — MissionConfig missing fields (PARTIALLY RESOLVED — one gap remains)

Partially resolved. The plan now includes:

- `shot_zone.altitude_surround` (plan line 152) — present.
- `bucket_height` (plan line 194) — present.
- `AirdropConfig` section covering `pid`, `limits`, `radius`, `accuracy`,
  `shot_duration`, `shot_wait`, `tar_z`, `shot_point_left/right`,
  `tar_pixel_left/right` (plan lines 155–169) — present.
- `LandingConfig` section covering `pid`, `limits`, `scout_halt`, `scout_x`,
  `scout_y`, `accuracy`, `tar_z`, `tar_pixel`, `descent_speed`,
  `descent_duration`, `surround_range`, `surround_step` (plan lines 171–184) —
  present.

**Remaining gap:** `MissionConfig` declares `heading_real_rad` (plan line 189)
but does not specify the YAML key name or the degree-to-radian conversion path.
The legacy `read_configs()` (OffboardControl.h lines 610–617) reads
`headingangle_real` as degrees from `OffboardControl.yaml` (now `mission.yaml`)
and converts via `headingangle_real * M_PI / 180.0`. The new `mission.yaml` does
not contain a `headingangle_real` key. The plan's `MissionConfig::load()`
description says it "reads mission.yaml + airdrop.yaml + landing.yaml" but does
not specify which key populates `heading_real_rad` or confirm that `mission.yaml`
carries `headingangle_real`.

Without this specification, the implementor either: (a) leaves `heading_real_rad`
at its 0.0 default, which differs from the legacy behavior where `headingangle_real`
defaults to `headingangle_compass` when the key is absent; or (b) adds the key to
`mission.yaml` but at the wrong unit (degrees vs. radians), producing a silent
numeric error.

### B-005 — DroneNode inherits NodeBase

Resolved. The plan now declares `class DroneNode : public NodeBase` (plan line
396). The `timer_callback()` pseudocode includes a system-status readiness guard
checking `motors_.connected()` and `motors_.armed()` (plan lines 453–455), plus
a GPS validity check (plan line 454). `fast_mode()` is accessible via the
inherited `NodeBase::fast_mode()`. No remaining gap.

### B-006 — Doland sub-state machine completeness

Resolved. The revised plan specifies five sub-states `{rtl, wait,
visual_approach, descent, land}` (plan line 375). Reviewing against the legacy
path:

- `rtl`: switch to RTL — matches StateMachine.cpp line 357.
- `wait`: wait 18 s, switch GUIDED — matches lines 361–365.
- `visual_approach`: load landing PID/limits; fly to `compass_to_world(scout_x,
  scout_y + 0.3)` at `scout_halt`; if no H target for > 2 s, execute linear
  ±3m search; if H detected, call `catch_target()` stub; timeout 19 s →
  descent — matches `OffboardControl::Doland()` lines 699–800.
- `descent`: send_velocity_timed(0, 0, -0.2, 0, 1.0) — matches line 804.
- `land`: switch LAND → `finished` — matches lines 377–380.

No remaining gap in behavior. However, see N-002 below regarding
`LandingConfig` missing `tar_x`/`tar_y`.

### B-007 — WaypointState reset protocol

Resolved. `WaypointState::reset()` is now documented (plan line 263). The
`timer_callback()` pseudocode shows `handler.reset()` called on state entry via
`prev_state_ != state_` detection (plan lines 457–465). Constraint C-8 codifies
this obligation. Invariant I-3 is clarified to apply only to
`frame_transforms.hpp`. No remaining gap.

---

## Remaining Issues

### R-B-004 (BLOCKER) — `heading_real_rad` has no YAML key and no conversion documented

- Severity: HIGH
- Section: `Spec / Data Structure / MissionConfig`
- Type: Correctness
- Problem:
  `MissionConfig` declares `heading_real_rad = 0` (plan line 189). The legacy
  code reads `headingangle_real` from `OffboardControl.yaml` in degrees and
  converts: `headingangle_real = config["headingangle_real"].as<float>(headingangle_compass)`.
  It then converts to radians: `headingangle_real = headingangle_real * M_PI / 180.0`.
  The default is `headingangle_compass`, not zero.

  The revised `mission.yaml` contains only `headingangle_compass: 349.0`. There
  is no `headingangle_real` key. The plan does not specify: (a) whether
  `mission.yaml` should add a `headingangle_real` key; (b) what the YAML key name
  is; or (c) that `MissionConfig::load()` must default `heading_real_rad` to
  `heading_compass_rad` when the key is absent.

  If the implementor leaves `heading_real_rad` at 0.0, the landing visual
  approach will rotate the scout offset by zero instead of the compass heading,
  sending the drone to the wrong search position.
- Fix:
  Specify in the `MissionConfig` struct comment and/or the `MissionConfig::load()`
  Step 1 description: add `headingangle_real` key to `mission.yaml` (optional,
  defaults to `headingangle_compass` when absent); convert degrees to radians on
  load; populate `heading_real_rad`. One additional line in Step 1 suffices.

### N-001 — `LandingConfig` is missing `tar_x` / `tar_y` pixel target fields

- Severity: MEDIUM
- Section: `Spec / Data Structure / MissionConfig / LandingConfig`
- Type: Completeness
- Problem:
  `OffboardControl::Doland()` (lines 711–715) reads `tar_x` and `tar_y` from
  `land_config.yaml`, defaulting to frame center when zero. The revised
  `landing.yaml` carries `tar_x: 640.0` and `tar_y: 400.0`. However,
  `LandingConfig` (plan lines 171–184) does not have `tar_x` / `tar_y` fields;
  it has `tar_pixel = {640, 400}` (plan line 179), which covers the same
  semantics but uses a different field name.

  This is a naming mismatch, not a missing field, but it must be made explicit:
  `MissionConfig::load()` must read `landing.yaml` keys `tar_x` and `tar_y` and
  store them into `landing.tar_pixel`. Without this explicit mapping in the plan,
  an implementor may read `tar_pixel` as a single YAML key that does not exist in
  the file.
- Fix:
  Add a YAML key mapping comment to `LandingConfig::tar_pixel`: `# tar_x, tar_y
  in landing.yaml`. Or add separate `float tar_x` and `float tar_y` fields with
  a note that they map to `landing.yaml` keys `tar_x` and `tar_y`.

### N-002 — `AirdropConfig` field names do not match `airdrop.yaml` keys

- Severity: MEDIUM
- Section: `Spec / Data Structure / MissionConfig / AirdropConfig`
- Type: Completeness
- Problem:
  The `AirdropConfig` struct (plan lines 155–169) declares `shot_point_left` and
  `shot_point_right` as `Eigen::Vector3f`. The `airdrop.yaml` file carries six
  scalar keys: `shot_target_x_l`, `shot_target_y_l`, `shot_target_z_l`,
  `shot_target_x_r`, `shot_target_y_r`, `shot_target_z_r`. Similarly,
  `tar_pixel_left = {665, 470}` and `tar_pixel_right = {615, 470}` in the struct
  correspond to keys `tar_x_l`, `tar_y_l`, `tar_x_r`, `tar_y_r` in the YAML.

  Without an explicit key mapping in the plan, the implementor must reverse-engineer
  the correspondence from the YAML file. The struct defaults match the YAML values,
  which suggests the mapping was intended but never written down.
- Fix:
  Add a comment block to `AirdropConfig` (or to Step 5 in the implementation plan)
  listing the YAML key names: `shot_target_x_l/y/z`, `shot_target_x_r/y/z`,
  `tar_x_l`, `tar_y_l`, `tar_x_r`, `tar_y_r`. This prevents a silent YAML parse
  failure where `load()` reads a non-existent key and uses the default value.

### N-003 — `surround_land` reset value inconsistency carried forward from legacy

- Severity: LOW
- Section: `Spec / Implement / LandingHandler sub-states`
- Type: Correctness (Documentation)
- Problem:
  The plan specifies `surround_land_ = -3` as the initial value (plan line 385)
  matching the legacy declaration (`static int surround_land = -3`). However the
  legacy `Doland()` resets `surround_land` to `0` in `LandState::end` (line 807),
  not to `-3`. If `LandingHandler::execute()` is called a second time (e.g., in
  testing or after a manual state override), `surround_land_` will start at 0
  instead of -3, skipping the negative-offset search positions.

  The plan's `reset()` method specification does not mention resetting
  `surround_land_` to `-3`. The member initializer sets `-3` only at construction.
- Why it matters:
  If `landing_.reset()` is called on state re-entry (as mandated by C-8), the
  sub-state reverts to `rtl` but `surround_land_` stays at 0 from a prior run.
  The search pattern covers [-3, -2, -1, 0, 1, 2, 3] in the legacy; a reset to 0
  covers only [0, 1, 2, 3], halving the search range.
- Fix:
  Document that `LandingHandler::reset()` must set `surround_land_ = -3`. Add
  this to the `reset()` method specification or add a comment in the struct
  member declaration.

---

## Positive Notes

- The two-timer design in `AirdropHandler` (`state_timer_` for the overall 70 s
  timeout, `wait_timer_` for per-barrel wait, `goto_timer_` for goto approach) is
  cleaner than the legacy's single shared `state_timer_` that gets reset at
  sub-state transitions. This is an improvement.
- The `local_to_world()` addition closes the B-003 gap precisely. The decision
  to use a single function with an `angle` parameter rather than splitting into
  compass and real variants is correct — callers pass the right angle explicitly,
  which is more testable.
- The `Subsystems` struct correctly uses references (`Motors &`, `InertialNav &`,
  `control::PosControl &`), satisfying C-6 and preventing shared_ptr overhead in
  handlers.
- V-UT-7 `fly_state_to_int encoding matches legacy values` is a critical test
  that was absent in Round 00. Including it prevents external ground station tools
  from breaking silently.
- The three-file config split (mission.yaml, airdrop.yaml, landing.yaml) maps
  cleanly to `AirdropConfig` and `LandingConfig` sub-structs. G-4 is achievable
  once the YAML key mappings are documented.
- Deferring `surround_shot_points` waypoint generation to config-driven YAML
  (rather than re-implementing the runtime normalization formula from
  `timer_callback()`) is the correct architectural decision for Phase 6 scope.

---

## Approval Conditions

### Must Fix Before Implementation

- R-B-004 — Specify the YAML key name for `heading_real_rad` in `mission.yaml`,
  document the degree-to-radian conversion, and document the default fallback to
  `heading_compass_rad` when the key is absent.

### Should Fix Before or During Implementation

- N-001 — Document YAML key mapping for `LandingConfig::tar_pixel` (`tar_x`,
  `tar_y` in `landing.yaml`).
- N-002 — Document YAML key mapping for `AirdropConfig::shot_point_left/right`
  and `tar_pixel_left/right`.

### Can Defer

- N-003 — Document `surround_land_ = -3` reset obligation in
  `LandingHandler::reset()`.

---

## Review Summary

| Severity | Count | Status |
|----------|-------|--------|
| CRITICAL | 0     | pass   |
| HIGH     | 1     | block  |
| MEDIUM   | 2     | warn   |
| LOW      | 1     | note   |

Verdict: BLOCKED on one HIGH issue (R-B-004). The fix is a single sentence
specifying the YAML key name and conversion for `heading_real_rad`. All 6 other
Round 00 blockers are verifiably resolved. Resolve R-B-004 and this plan is
cleared for implementation.
