# `config-testing` PLAN `00`

> Status: Draft
> Feature: `config-testing` (Phase 7 + Phase 8 combined)
> Iteration: `00`
> Owner: Planner
> Dependencies:
>
> - Phases 0-6 complete (all modules compiled, 203 tests passing)

---

## Log

### Feature Introduce

Phases 7 and 8 are combined because they are tightly coupled: config consolidation
(Phase 7) enables integration testing, and CI improvements (Phase 8) validate the
config + launch infrastructure. Both are small scope compared to earlier phases.

Current state:
- 6 config YAML files, no launch files, no main.cpp, no executable
- 16 test files (203 tests), all unit-level, no integration tests
- CI has format check + build/test on ROS Humble
- Docker workflow for local builds
- No clang-tidy, no coverage reporting

Phase 7 adds: main.cpp executable, ROS 2 launch file, config consolidation notes.
Phase 8 adds: additional tests for untested modules, clang-tidy CI, coverage target.

### Changes from Previous Round

- N/A (first iteration)

---

## Spec

### Goals

- G-1: Create `src/main.cpp` entry point that instantiates `DroneNode`
- G-2: Create a ROS 2 launch file with configurable parameters
- G-3: Add TensorRT model path and camera device to config
- G-4: Add unit tests for modules missing coverage (drivers, servo, gimbal are not unit-testable without ROS mock, but node_base params can be)
- G-5: Add clang-tidy check to CI
- G-6: Reach 80%+ test coverage on math, control, perception, and mission utilities
- G-7: Document all ROS 2 parameters and topics

### Non-goals

- NG-1: End-to-end testing on real Jetson (requires hardware)
- NG-2: Mock MAVROS infrastructure (too complex for this phase, deferred)
- NG-3: Consolidating 6 YAML files into 1 (breaks existing config loading paths)
- NG-4: Running TensorRT inference in CI (no GPU)
- NG-5: Changing any flight behavior or mission logic

### Architecture

```
src/
  main.cpp                  # NEW: entry point, creates DroneNode

launch/
  drone.launch.py           # NEW: ROS 2 launch file

config/
  mission.yaml              # Existing, add camera_device + model_path fields
  camera.yaml               # Existing
  pos_control.yaml          # Existing
  airdrop.yaml              # Existing
  landing.yaml              # Existing
  position.yaml             # Existing (legacy, kept for reference)

test/
  test_basic_pid.cpp        # NEW: BasicPID tests
  test_scurve.cpp           # NEW: SCurve basic tests
  test_waypoint_nav.cpp     # NEW: WaypointState reset, waypoint progression

.github/workflows/ci.yml   # Updated: add clang-tidy job
.clang-tidy                 # NEW: tidy config
```

### Invariants

- I-1: main.cpp is minimal (< 30 lines), only creates node and spins
- I-2: Launch file supports parameter overrides for all config paths
- I-3: No test depends on hardware (MAVROS, camera, GPU)
- I-4: CI must pass in under 10 minutes
- I-5: All new tests are pure unit tests linking only library targets

### Data Structure

#### File Triage

| Item | Action | Notes |
|---|---|---|
| main.cpp | Create | Instantiate DroneNode, spin |
| drone.launch.py | Create | mavros_ns, config_dir as launch args |
| camera_device config | Add | `camera_device: /dev/video0` in mission.yaml |
| model_path config | Add | `model_path: model/best.engine` in mission.yaml |
| test_basic_pid.cpp | Create | Tests for BasicPID (Phase 2, untested) |
| test_scurve.cpp | Create | Basic SCurve tests (Phase 2, untested) |
| test_waypoint_nav.cpp | Create | WaypointState reset, progression |
| .clang-tidy | Create | bugprone, cert, cppcoreguidelines checks |
| ci.yml clang-tidy job | Update | Add tidy step after build |

### API Surface

```cpp
// --- main.cpp ---
#include "drone/mission/drone_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone::mission::DroneNode>("/mavros/", "config");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

```python
# --- launch/drone.launch.py ---
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('drone')
    return LaunchDescription([
        DeclareLaunchArgument('mavros_ns', default_value='/mavros/'),
        DeclareLaunchArgument('config_dir', default_value=pkg_dir + '/config'),
        Node(
            package='drone',
            executable='drone_node',
            name='drone_node',
            parameters=[{
                'mavros_ns': LaunchConfiguration('mavros_ns'),
                'config_dir': LaunchConfiguration('config_dir'),
            }],
            output='screen',
        ),
    ])
```

### Constraints

- C-1: main.cpp must not contain mission logic, only node construction and spin
- C-2: Launch file must be installable via ament_cmake
- C-3: clang-tidy must not break existing CI pass (warnings only initially)
- C-4: New tests must not increase CI time by more than 30 seconds
- C-5: Config additions (camera_device, model_path) must have sensible defaults

---

## Implement

### Implementation Plan

#### Step 1: Create `src/main.cpp`

- Minimal entry point: rclcpp::init, make_shared DroneNode, spin, shutdown
- DroneNode constructor takes mavros_ns and config_dir
- Note: DroneNode inherits NodeBase which inherits rclcpp::Node, so make_shared works

#### Step 2: Update CMakeLists.txt for executable

- Add `add_executable(drone_node src/main.cpp)`
- Link against `drone_mission`
- Install executable to `lib/${PROJECT_NAME}`

#### Step 3: Create `launch/drone.launch.py`

- Declare launch arguments: mavros_ns, config_dir
- Use ament_index to find package share directory for default config_dir
- Node action with parameter passing

#### Step 4: Update CMakeLists.txt for launch install

- `install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)`
- Add `ros2launch` as test dependency in package.xml

#### Step 5: Add config fields to mission.yaml

- `camera_device: /dev/video0`
- `model_path: model/best.engine`
- These are read by Phase 7 integration (not consumed yet, but defined)

#### Step 6: Create `.clang-tidy`

- Enable: bugprone-*, cert-*, cppcoreguidelines-*, performance-*, readability-*
- Disable noisy checks: readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers
- Disable checks incompatible with ROS 2 style

#### Step 7: Update CI for clang-tidy

- Add clang-tidy run after successful build (warnings only, non-blocking initially)
- Use `run-clang-tidy` with compile_commands.json from colcon build

#### Step 8: Write new unit tests

- `test_basic_pid.cpp`: construct, compute, output limits, reset
- `test_scurve.cpp`: basic trajectory generation, motion complete detection
- `test_waypoint_nav.cpp`: WaypointState reset, empty waypoints returns true

#### Step 9: Update package.xml

- Add `exec_depend` for `ros2launch` if using launch files
- Add `build_depend` for `ament_cmake_clang_tidy` if available

#### Step 10: Update PROGRESS.md

### Trade-offs

#### T-1: Config consolidation (merge 6 files into 1) vs keep separate

- Decision: Keep separate. MissionConfig::load() already reads 3 files cleanly.
- Rationale: Merging changes the YAML schema, breaks any external tools that
  read individual config files, and adds no real value. NG-3.

#### T-2: clang-tidy blocking vs warning-only

- Decision: Warning-only in CI initially. Too many existing false positives
  from ROS 2 generated code and Eigen headers.
- Rationale: Adding it as blocking would require fixing hundreds of
  warnings from external headers. Better to add gradually.

#### T-3: Coverage reporting tool

- Decision: Use lcov with colcon test if available, otherwise manual check.
- Rationale: Coverage CI integration is complex with ament. Focus on writing
  tests that cover untested code paths.

---

## Validation

### Unit Tests

- V-UT-1: BasicPID construct with default gains produces zero output
- V-UT-2: BasicPID compute tracks target
- V-UT-3: BasicPID output respects limits
- V-UT-4: BasicPID reset clears state
- V-UT-5: SCurve generates trajectory from A to B
- V-UT-6: SCurve returns complete when motion finished
- V-UT-7: WaypointState reset clears index and timer
- V-UT-8: waypoint_goto_next with empty waypoints returns true immediately

### Integration Tests

- V-IT-1: main.cpp compiles and links successfully
- V-IT-2: Launch file syntax is valid (ros2 launch --show-args)
- V-IT-3: All headers compile standalone

### Acceptance Mapping

| Goal | Validation |
|------|------------|
| G-1 main.cpp | V-IT-1, executable installs |
| G-2 Launch file | V-IT-2, launch installs |
| G-3 Config fields | mission.yaml updated |
| G-4 New tests | V-UT-1..8 |
| G-5 clang-tidy CI | CI job present (non-blocking) |
| G-6 80%+ coverage | 8 new tests on untested modules |
| G-7 Documentation | Launch args documented in launch file |
