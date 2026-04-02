# `Utilities` PLAN `02`

> Status: Draft
> Feature: `utilities`
> Iteration: `02`
> Owner: Executor
> Depends on:
> - Previous Plan: `01_PLAN.md`
> - Review: `01_REVIEW.md`
> - Master Directive: `01_MASTER.md`

---

## Summary

Transfer leaf-level utility modules into `Drone` with public headers in
`include/drone/` for cross-package reuse (future `ros2_yolov8` integration).
Fix readyaml config path to use ament installed layout. Complete dependency
declarations in both CMake and package.xml.

## Log

[**Feature Introduce**]

Switched from internal-only `src/` headers to public `include/drone/` layout.
This supports future cross-package consumers (e.g. `ros2_yolov8` detecting
targets and publishing to `drone` topics, or sharing common types).

[**Review Adjustments**]

- R-001 (HIGH): Fixed. `readyaml.hpp` now resolves config from the ament
  installed layout: `get_package_share_directory("drone") + "/config/" + filename`.
  Removed the legacy `../../../../src/` relative path hack.
- R-002 (MEDIUM): Fixed. Added `<depend>yaml-cpp</depend>` to package.xml.
- R-003 (LOW): Fixed. Added `find_package(eigen3_cmake_module REQUIRED)`
  before `find_package(Eigen3 REQUIRED)` in CMake.

[**Master Compliance**]

No new master directives. Previous directives (M-001/M-002/M-003 from round 00)
remain applied.

### Changes from Previous Round

[**Added**]
- Public header layout `include/drone/` for cross-package support.
- Header install rules in CMakeLists.txt.
- Library export rules (`ament_export_targets`, `ament_export_dependencies`).
- `yaml-cpp` in package.xml.
- `eigen3_cmake_module` in CMake find_package.
- Fixed readyaml config path resolution.

[**Changed**]
- Headers moved from `src/` to `include/drone/`.
- Library model: internal-only → exported (for future `ros2_yolov8` consumer).

[**Removed**]
Nothing.

[**Unresolved**]
Nothing.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | readyaml uses ament installed layout |
| Review | R-002 | Accepted | yaml-cpp added to package.xml |
| Review | R-003 | Accepted | eigen3_cmake_module in CMake |

---

## Spec

[**Goals**]

- G-1: Transfer `readyaml`, `timer`, `keyboard`, `rotate`, `fuzzy_pid`,
  `autotune`, and `trajectory_generator` into `Drone`.
- G-2: All code compiles as an exported library, C++20, zero warnings.
- G-3: File naming: snake_case `.hpp`/`.cpp`.
- G-4: `readyaml` resolves config from ament installed layout.
- G-5: Headers installable for cross-package use.

Non-goals:
- NG-1: No math layer — deferred per roadmap.
- NG-2: No logic refactoring — transfer only.
- NG-3: No unit tests.

[**Architecture**]

```
Drone/
├── include/drone/
│   ├── utils/
│   │   ├── readyaml.hpp
│   │   ├── timer.hpp
│   │   ├── keyboard.hpp
│   │   └── rotate.hpp
│   ├── control/
│   │   ├── fuzzy_pid.hpp
│   │   └── autotune.hpp
│   └── trajectory/
│       └── trajectory_generator.hpp
├── src/
│   ├── control/
│   │   ├── fuzzy_pid.cpp
│   │   └── autotune.cpp
│   └── trajectory/
│       └── trajectory_generator.cpp
├── CMakeLists.txt
├── package.xml
└── ...
```

Headers in `include/drone/` — public, installable.
Sources in `src/` — private, not installed.

[**Invariants**]

- I-1: No `using namespace` at file scope in any header.
- I-2: All headers use `#pragma once`.
- I-3: Every header is self-contained.
- I-4: `readyaml.hpp` resolves config via `ament_index_cpp`, not relative paths.
- I-5: `Readyaml::getPackageName()` returns `"drone"`.
- I-6: No behavioral changes except readyaml path resolution.

[**Data Structure**]

| Legacy File | Target Header | Target Source | Changes |
|---|---|---|---|
| `Readyaml.h` | `include/drone/utils/readyaml.hpp` | — (header-only) | Package name, config path fix |
| `utils.h` (Timer) | `include/drone/utils/timer.hpp` | — | Extract Timer class |
| `utils.h` (_kbhit/_getch) | `include/drone/utils/keyboard.hpp` | — | Extract keyboard fns |
| `utils.h` (rotate_angle) | `include/drone/utils/rotate.hpp` | — | Extract template |
| `FuzzyPID.h` | `include/drone/control/fuzzy_pid.hpp` | `src/control/fuzzy_pid.cpp` | Remove `using namespace std` |
| `FuzzyPID.cpp` | — | `src/control/fuzzy_pid.cpp` | Include path update |
| `AutoTune.h` | `include/drone/control/autotune.hpp` | `src/control/autotune.cpp` | `#pragma once` |
| `AutoTune.cpp` | — | `src/control/autotune.cpp` | Include path update |
| `TrajectoryGenerator.h` | `include/drone/trajectory/trajectory_generator.hpp` | `src/trajectory/trajectory_generator.cpp` | `#pragma once` |
| `TrajectoryGenerator.cpp` | — | `src/trajectory/trajectory_generator.cpp` | Include path update |

[**API Surface**]

All function signatures unchanged. Only changes:
- `Readyaml::getPackageName()` returns `"drone"`.
- `Readyaml::readYAML()` resolves from `share/drone/config/` instead of
  source tree relative path.

[**Constraints**]

- C-1: No dependency on `OffboardControl.h` or any ROS node class.
- C-2: FuzzyPID macros preserved as-is.
- C-3: Headers installed to `include/drone/` for cross-package access.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create directory structure.
2. Transfer each module.
3. Fix readyaml config path.
4. Update CMakeLists.txt.
5. Update package.xml.
6. Build and verify.

[**Failure Flow**]

1. Compile errors → fix includes.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — Directories**]

```
mkdir -p include/drone/utils include/drone/control include/drone/trajectory
mkdir -p src/control src/trajectory
rm src/.gitkeep
```

[**Step 2 — Transfer modules**]

Mechanical transfer with:
- `#pragma once`
- Remove `using namespace` from headers
- Update include paths to `drone/utils/...`, `drone/control/...`, etc.

[**Step 3 — Fix readyaml**]

```cpp
// Before (legacy):
std::string relative_path = package_share_directory +
    "/../../../../src/" + getPackageName() + "/config/" + filename;
std::filesystem::path config_file_path = std::filesystem::canonical(relative_path);

// After:
std::filesystem::path config_file_path =
    ament_index_cpp::get_package_share_directory(getPackageName()) +
    "/config/" + filename;
```

[**Step 4 — CMakeLists.txt**]

```cmake
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(drone_utils
  src/control/fuzzy_pid.cpp
  src/control/autotune.cpp
  src/trajectory/trajectory_generator.cpp
)

target_include_directories(drone_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(drone_utils yaml-cpp)
ament_target_dependencies(drone_utils Eigen3 ament_index_cpp)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library
install(TARGETS drone_utils
  EXPORT export_drone_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_drone_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 ament_index_cpp)
```

[**Step 5 — package.xml**]

Add:
```xml
<buildtool_depend>eigen3_cmake_module</buildtool_depend>
<depend>eigen</depend>
<depend>ament_index_cpp</depend>
<depend>yaml-cpp</depend>
```

[**Step 6 — Build**]

`colcon build --packages-select drone` — zero warnings.

## Trade-offs

- T-1: **`include/drone/` vs `src/` headers**
  - `include/drone/`: standard ROS 2 public header layout. Required for
    cross-package consumers like `ros2_yolov8`.
  - `src/`: simpler but blocks future package composition.
  - Decision: `include/drone/`. Future `ros2_yolov8` integration requires it.

---

## Validation

[**Unit Tests**]

- V-UT-1: Deferred (NG-3).

[**Integration Tests**]

- V-IT-1: `colcon build --packages-select drone` succeeds, zero warnings.
- V-IT-2: `colcon test --packages-select drone` passes lint.
- V-IT-3: Library target exists in build output.
- V-IT-4: `install/drone/include/drone/` contains all `.hpp` headers.

[**Failure / Robustness Validation**]

- V-F-1: Each `.hpp` compiles standalone.

[**Edge Case Validation**]

- V-E-1: `readyaml.hpp` resolves config from `share/drone/config/`.
- V-E-2: `grep -r "using namespace" include/` returns zero results.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1, V-IT-3 |
| G-2 | V-IT-1 (zero warnings) |
| G-3 | All files snake_case .hpp/.cpp |
| G-4 | V-E-1 |
| G-5 | V-IT-4 |
| C-1 | V-F-1 |
| C-2 | FuzzyPID macros in fuzzy_pid.hpp |
| C-3 | V-IT-4 (headers installed) |
