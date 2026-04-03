# `config-testing` REVIEW `00`

> Status: Approved for Implementation
> Feature: `config-testing` (Phase 7 + Phase 8 combined)
> Iteration: `00`
> Owner: Reviewer
> Target Plan: `00_PLAN.md`

---

## Verdict

- Decision: Approved for Implementation
- Blocking Issues: `0`
- Non-Blocking Issues: `4`

## Summary

The plan is structurally sound and ready to implement. The main.cpp entry
point, CMake executable setup, launch file, and new test targets are all
correctly specified. Four non-blocking issues are documented below — none
will cause build failures or incorrect behavior, but two (N-001, N-002) should
be resolved during implementation to avoid surprises.

---

## Scope-by-Scope Findings

### 1. Plan Correctness — main.cpp, Launch File, Executable Setup

**Assessment: Correct.**

The proposed `main.cpp` (plan lines 113-121) is minimal and correct:

```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<drone::mission::DroneNode>("/mavros/", "config");
rclcpp::spin(node);
rclcpp::shutdown();
```

`DroneNode` inherits `NodeBase` which inherits `rclcpp::Node`. The
`rclcpp::Node` constructor is public and `DroneNode` has no deleted copy/move
constructors, so `make_shared<DroneNode>(...)` compiles correctly. The
two-argument constructor `DroneNode(const std::string & mavros_ns,
const std::string & config_dir)` is verified in
`include/drone/mission/drone_node.hpp` line 30 and the implementation in
`src/mission/drone_node.cpp` line 12 confirms both arguments are forwarded
correctly into `NodeBase` and subsystem constructors.

The launch file Python syntax (plan lines 125-148) is valid ROS 2 Humble
launch syntax. `get_package_share_directory('drone')` correctly resolves the
installed `config/` directory (CMakeLists.txt line 159 installs
`config/ → share/${PROJECT_NAME}/config`). The node executable name
`drone_node` matches the `add_executable(drone_node ...)` target name
proposed in Step 2.

**One precision issue on the default config_dir**: The plan sets
`default_value=pkg_dir + '/config'`. After `colcon install`, the config
directory is at `share/drone/config`, so `get_package_share_directory('drone')`
returns the `share/drone` share dir and `+ '/config'` produces the correct
path. This is correct.

---

### 2. CMake Integration — Executable and Launch Install

**Assessment: Correct, with one minor gap.**

The proposed CMake additions are:

```cmake
add_executable(drone_node src/main.cpp)
target_link_libraries(drone_node drone_mission)
install(TARGETS drone_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
```

This follows the ament_cmake convention exactly. The existing
`drone_mission` library target (CMakeLists.txt lines 136-155) already
links `drone_drivers`, `drone_control`, and `drone_utils` transitively, so
`drone_node` executable will have full link closure from a single
`target_link_libraries` call.

The `install(TARGETS drone_node DESTINATION lib/${PROJECT_NAME})` line
should be added to the existing `install(TARGETS ...)` block (CMakeLists.txt
lines 161-166) or as a separate call — both work with ament_cmake, but
keeping executables in the same block as libraries is the project convention.

**N-001 (LOW): `drone_node` target not added to `ament_export_targets`**

The current `ament_export_targets(export_drone HAS_LIBRARY_TARGET)`
(CMakeLists.txt line 177) exports the library targets. The `drone_node`
executable does not need to be exported (no downstream package consumes it),
but if the implementor adds it to the existing `install(TARGETS ...)` block
that feeds `export_drone`, CMake will accept it. If they create a separate
`install(TARGETS drone_node ...)` without `EXPORT`, that is also correct.
The plan does not specify which pattern to use — either works, but the
separate `install()` pattern (no EXPORT) is cleaner for executables.
No fix required, just awareness.

---

### 3. Config Completeness — New Fields in mission.yaml

**Assessment: Correct placement, with one structural note.**

Adding `camera_device: /dev/video0` and `model_path: model/best.engine` to
`config/mission.yaml` is the correct file for device-level configuration
since `MissionConfig::load()` reads `mission.yaml` as its primary source
(drone_node.cpp line 20-23).

However, `MissionConfig` struct (mission_types.hpp lines 118-141) does not
currently have `camera_device` or `model_path` fields. The plan acknowledges
this: "These are read by Phase 7 integration (not consumed yet, but defined)"
(Step 5). Adding YAML keys that no struct field reads is safe — YAML-cpp
silently ignores unknown keys unless the loader explicitly iterates all keys.
`MissionConfig::load()` reads by name (`config["field"].as<T>()`), so
unread keys are harmless.

**N-002 (MEDIUM): Config fields belong in camera.yaml, not mission.yaml**

`camera_device` and `model_path` are perception-layer configuration. The
project already has `config/camera.yaml` (listed in the architecture table)
which is the natural home for these fields. Placing them in `mission.yaml`
creates a mixed-layer concern: `MissionConfig` is consumed by
`drone_node.cpp` (mission layer), but camera and TensorRT config belongs to
`CameraDriver`/`Detector` (perception layer).

This does not break anything in Phase 7+8 since neither field is consumed
yet, but when Phase 3 perception components eventually read these values, they
will have to reach into the mission config path. The fix is to add them to
`config/camera.yaml` instead, which `CameraDriver` is already expected to
load. The plan should be updated before implementation.

---

### 4. Test Coverage — New Unit Tests

**Assessment: Mostly correct. One test target has a linkage risk.**

#### test_basic_pid.cpp → links drone_utils

`BasicPID` (basic_pid.hpp lines 1-70) is a header-only class with no
corresponding `.cpp` file. It depends on `drone/utils/readyaml.hpp` and
`yaml-cpp`. The `drone_utils` library (CMakeLists.txt lines 29-44) links
`yaml-cpp` publicly, so `test_basic_pid` linking only `drone_utils` will
have full dependency closure. The proposed test cases (V-UT-1 through V-UT-4)
are testable without ROS or hardware.

One precision point: `BasicPID::readPIDParameters()` uses `ConfigLoader::load()`
which reads a file from disk. Test V-UT-1 ("construct with default gains
produces zero output") requires no file I/O since `BasicPID()` default
constructs with `kp_ = ki_ = kd_ = 0`. For V-UT-2 and V-UT-3 (compute,
limits), the test either calls `readPIDParameters` with a test fixture file
or sets the public fields directly (`kp_`, `ki_`, `output_limit_` are all
public in basic_pid.hpp lines 61-63). Direct field assignment is simpler and
avoids test file I/O.

#### test_scurve.cpp → links drone_utils

`SCurve` has a `.cpp` implementation (`src/math/scurve.cpp`) that is compiled
into `drone_utils` (CMakeLists.txt line 35). Linking `drone_utils` is
correct.

**N-003 (HIGH): SCurve test interface is incompatible with proposed test cases**

The plan proposes V-UT-5: "SCurve generates trajectory from A to B" and
V-UT-6: "SCurve returns complete when motion finished." However, examining
the `SCurve` public API (scurve.hpp lines 16-83):

- `calculate_track()` requires 9 parameters including origin, destination,
  and kinematic limits. It does not return a result — it populates internal
  state.
- `advance_target_along_track()` requires three `SCurve &` arguments
  (`prev_leg`, `next_leg`, current) plus a `WaypointState`-like accumulator.
  This is the `TrajectoryController` internal integration interface, not a
  standalone "call once and check finished" API.
- `finished()` exists (line 36) but it tests the internal segment time, not
  end-to-end trajectory completion.

The test cases as described in V-UT-5 and V-UT-6 imply a simpler interface
than `SCurve` actually exposes. The existing `test_trajectory_controller.cpp`
(which links `drone_control`) likely already covers `SCurve` indirectly via
`TrajectoryController`. Writing a `test_scurve.cpp` that calls
`calculate_path()` (the static method, line 17) and validates the output
parameters is achievable and does add genuine coverage. But
`advance_target_along_track()` requires a multi-leg setup that is not
practically unit-testable without significant scaffolding.

The implementor should scope `test_scurve.cpp` to the static `calculate_path()`
method and `finished()` after `calculate_track()` + manual `advance_time()`
calls, not to the full `advance_target_along_track()` pipeline. The plan's
test descriptions should be more precise about which methods are exercised.

#### test_waypoint_nav.cpp → links drone_mission

`WaypointState` (waypoint_nav.hpp lines 10-22) is a plain struct with a
`reset()` method. `waypoint_goto_next()` is a free function taking `Subsystems &`
(mission_types.hpp lines 64-71) — which requires references to `Motors`,
`InertialNav`, `PosControl`, `Servo`, and `Gimbal`. All of these are ROS 2
nodes or depend on `rclcpp::Node`.

**N-004 (MEDIUM): test_waypoint_nav.cpp cannot test waypoint_goto_next without ROS infrastructure**

V-UT-7 ("WaypointState reset clears index and timer") is testable — it only
constructs `WaypointState` and calls `reset()`. This requires linking
`drone_mission` but no ROS spin.

V-UT-8 ("waypoint_goto_next with empty waypoints returns true immediately")
requires constructing a valid `Subsystems` struct, which requires live
`Motors`, `InertialNav`, `PosControl`, `Servo`, and `Gimbal` objects. Each of
these requires an `rclcpp::Node &` in their constructor. The test would need
to call `rclcpp::init()` and construct a dummy node, which makes it an
integration test, not a unit test. This violates invariant I-3 ("No test
depends on hardware") and I-5 ("All new tests are pure unit tests linking
only library targets").

The plan's constraint I-5 and the proposed test V-UT-8 are in direct
conflict. V-UT-8 should either be dropped or scoped to only V-UT-7 (the
`WaypointState::reset()` path). The `waypoint_goto_next` function itself
cannot be exercised without a `Subsystems` reference.

Recommended scope for `test_waypoint_nav.cpp`:
- V-UT-7: Construct `WaypointState`, verify default values, call `reset()`,
  verify fields cleared. Links `drone_mission`.
- Drop V-UT-8 as stated. The empty-waypoints path in `waypoint_goto_next`
  is covered indirectly by `test_fly_state.cpp` or mission handler tests.

---

### 5. CI Improvements — clang-tidy Approach

**Assessment: Practical as specified (warning-only). Two process notes.**

The trade-off T-2 decision (warning-only initially) is the correct call.
The Eigen and ROS 2 generated headers produce hundreds of false-positive
tidy warnings that are unfixable from user code. Adding `--header-filter` to
scope tidy to only project headers (`include/drone/.*`) would reduce noise
significantly and is worth specifying in the `.clang-tidy` or the CI invocation.

The plan mentions using `run-clang-tidy` with `compile_commands.json` from
`colcon build`. For this to work, `colcon build` must be invoked with
`--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. The existing CI `build`
step (ci.yml line 61) does not pass this flag. The clang-tidy CI job must
either add this flag or use a separate build step.

The `ament_cmake_clang_tidy` package (Step 9) is not available in the
standard ROS 2 Humble apt repositories. The CI Dockerfile and ci.yml
dependency list both omit it. Using `run-clang-tidy` directly (from the
`clang-tools` apt package) against `compile_commands.json` is simpler and
does not require `ament_cmake_clang_tidy`. The plan's Step 9 note "if
available" correctly hedges this, but the CI job implementation should use
`run-clang-tidy` directly to avoid a conditional that complicates the CI
YAML.

---

### 6. DroneNode Compatibility — make_shared Construction

**Assessment: Confirmed compatible.**

`DroneNode` inherits `NodeBase` → `rclcpp::Node`. The `DroneNode(const
std::string &, const std::string &)` constructor is public (drone_node.hpp
line 30). `rclcpp::Node` uses the `NodeFactory` pattern internally but does
not delete the copy constructor in a way that blocks `make_shared`. The
standard ROS 2 pattern for node construction is precisely
`std::make_shared<MyNode>(args...)`, and the DroneNode constructor (verified
in drone_node.cpp lines 12-29) has no static assertions or
`enable_shared_from_this` complications that would break `make_shared`.

The construction chain is:
1. `NodeBase("drone_node", mavros_ns)` initializes `rclcpp::Node`
2. Member subsystems `motors_`, `inav_`, `servo_`, `gimbal_` take `*this`
   (the `NodeBase` / `rclcpp::Node` reference) — this is safe because all
   members are initialized after the base class.
3. `pos_control_`, `subs_`, `config_`, and handlers follow in order.

No issues with make_shared construction.

---

## Positive Notes

- Combining Phase 7 and Phase 8 into a single small phase is the right
  scoping decision. Neither phase alone is large enough to warrant a full
  iteration cycle.
- Keeping the 6 YAML files separate (NG-3) is correct. The current
  `MissionConfig::load()` signature takes three paths explicitly, and
  consolidation would break that interface without benefit.
- The `.clang-tidy` check set (bugprone, cert, cppcoreguidelines, performance,
  readability) with magic-numbers disabled is a well-calibrated starting
  configuration for a ROS 2 C++ codebase.
- The invariant I-1 (main.cpp < 30 lines) and I-2 (launch file supports
  parameter overrides) are correctly enforceable from the proposed
  implementation.
- The `BasicPID` test design is sound — public field access avoids file I/O
  in unit tests, which matches invariant I-3 well.
- Deferring Mock MAVROS (NG-2) is the right call. The complexity cost of
  mocking the full MAVROS subscription infrastructure exceeds the coverage
  benefit for this final phase.

---

## Approval Conditions

### Must Fix Before Implementation

None. No blocking issues found.

### Should Fix During Implementation

- **N-002** — Move `camera_device` and `model_path` config fields to
  `config/camera.yaml`, not `config/mission.yaml`. Perception-layer config
  belongs with the perception config file.
- **N-003** — Scope `test_scurve.cpp` to the static `calculate_path()`
  method and `finished()` after `advance_time()`. Do not attempt to unit-test
  `advance_target_along_track()` — it requires multi-leg scaffolding that
  belongs in integration testing.
- **N-004** — Drop V-UT-8 from `test_waypoint_nav.cpp`. Testing
  `waypoint_goto_next()` requires a live `Subsystems` struct (ROS nodes),
  which violates I-3 and I-5. Keep only V-UT-7 (`WaypointState::reset()`).

### Note for Implementation

- **N-001** — For the clang-tidy CI job, add
  `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to the colcon build
  invocation and use `run-clang-tidy` directly with a `--header-filter` of
  `include/drone/.*` to limit noise from Eigen and ROS 2 generated headers.

---

## Review Summary

| Severity | Count | Status |
|----------|-------|--------|
| CRITICAL | 0     | pass   |
| HIGH     | 1     | warn   |
| MEDIUM   | 2     | info   |
| LOW      | 1     | note   |

Verdict: APPROVED — No blocking issues. The HIGH finding (N-003) on SCurve
test scope and the MEDIUM finding (N-004) on waypoint_goto_next testability
must both be resolved during implementation to avoid writing tests that either
cannot compile or silently test the wrong scope. The plan is otherwise
complete and consistent with the existing codebase structure.
