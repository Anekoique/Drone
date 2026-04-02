# `Utilities` PLAN `01`

> Status: Draft
> Feature: `utilities`
> Iteration: `01`
> Owner: Executor
> Depends on:
> - Previous Plan: `00_PLAN.md`
> - Review: `00_REVIEW.md`
> - Master Directive: `00_MASTER.md`

---

## Summary

Transfer leaf-level utility modules from the legacy codebase into the `Drone`
project. Minimal code changes — only what is required for compilation under the
new package name and C++20. No logic refactoring, no tests, no math layer
(deferred per roadmap).

## Log

[**Feature Introduce**]

Scoped down to a clean mechanical transfer. Uses modern ROS 2 file naming
conventions (snake_case `.hpp`/`.cpp`). Internal-only library, no install/export.

[**Review Adjustments**]

- R-001 (HIGH): Accepted. Removed `math_utils` from this feature entirely.
  The full math helper layer (`math_utils.h` + `math.h` + `math.cpp`) will
  migrate together in a later feature as the roadmap specifies.
- R-002 (HIGH): Accepted. Library is explicitly internal-only. Headers stay in
  `src/`, no `$<INSTALL_INTERFACE>`, no export rules. `yaml-cpp` linked via
  `target_link_libraries`.
- R-003 (MEDIUM): Accepted. Dead-code deletion removed from scope. Legacy tree
  is untouched by this feature.
- R-004 (MEDIUM): Noted. Per M-003, this is a transfer phase — no semantic
  changes that need test coverage. Tests deferred to a testing feature.

[**Master Compliance**]

- M-001: Strictly follows roadmap Phase 1 scope. No math layer, no PID, no
  modules from other phases.
- M-002: File naming follows modern ROS 2 conventions — snake_case, `.hpp`
  for headers, `.cpp` for sources. Directory layout mirrors nav2_util pattern.
- M-003: Minimal code changes. Only: `#pragma once`, package name fix in
  readyaml, `using namespace` removal from headers. No logic modifications.

### Changes from Previous Round

[**Added**]
- Explicit internal-only library model.
- `.hpp` header extension per modern ROS 2 convention.

[**Changed**]
- Removed `math_utils` from scope (R-001, roadmap alignment).
- Removed dead-code deletion from scope (R-003).
- CMake model: internal library, no install/export, `yaml-cpp` via
  `target_link_libraries`.
- Reduced code changes to mechanical transfer only (M-003).

[**Removed**]
- `math_utils.h/cpp` migration (deferred to math feature).
- `AC_WPNav`, `.bak` file cleanup (not in Drone tree, out of scope).
- Unit tests (deferred).
- Comment translation beyond what is necessary for clarity.

[**Unresolved**]
Nothing.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | math_utils removed from scope |
| Review | R-002 | Accepted | Internal-only library, yaml-cpp via target_link_libraries |
| Review | R-003 | Accepted | Dead-code deletion removed from scope |
| Review | R-004 | Noted | Per M-003, transfer only — tests deferred |
| Review | TR-1 | Accepted | Internal-only model |
| Review | TR-2 | Accepted | Math layer deferred entirely |
| Master | M-001 | Applied | Scope matches roadmap Phase 1 exactly |
| Master | M-002 | Applied | snake_case .hpp/.cpp per nav2/autoware convention |
| Master | M-003 | Applied | Mechanical transfer, no logic changes |

---

## Spec

[**Goals**]

- G-1: Transfer `readyaml`, `timer`, `keyboard`, `rotate`, `fuzzy_pid`,
  `autotune`, and `trajectory_generator` into `src/`.
- G-2: All code compiles as an internal static library, C++20, zero warnings.
- G-3: File naming follows modern ROS 2 conventions (snake_case `.hpp`/`.cpp`).
- G-4: `Readyaml` references package name `drone`.

Non-goals:
- NG-1: No math layer (math_utils.h, math.h) — deferred per roadmap.
- NG-2: No logic refactoring — transfer only with minimal mechanical fixes.
- NG-3: No unit tests.
- NG-4: No legacy tree modifications.

[**Architecture**]

```
src/
├── utils/
│   ├── readyaml.hpp           (from Readyaml.h)
│   ├── timer.hpp              (from utils.h Timer class)
│   ├── keyboard.hpp           (from utils.h _kbhit/_getch)
│   └── rotate.hpp             (from utils.h rotate_angle)
├── control/
│   ├── fuzzy_pid.hpp          (from FuzzyPID.h)
│   ├── fuzzy_pid.cpp          (from FuzzyPID.cpp)
│   ├── autotune.hpp           (from AutoTune.h)
│   └── autotune.cpp           (from AutoTune.cpp)
└── trajectory/
    ├── trajectory_generator.hpp (from TrajectoryGenerator.h)
    └── trajectory_generator.cpp (from TrajectoryGenerator.cpp)
```

[**Invariants**]

- I-1: No `using namespace` at file scope in any header.
- I-2: All headers use `#pragma once`.
- I-3: Every header is self-contained.
- I-4: No module depends on another project-local module outside its directory,
  except `readyaml.hpp` which is a shared utility.
- I-5: `Readyaml::getPackageName()` returns `"drone"`.
- I-6: No behavioral changes to any function.

[**Data Structure**]

| Legacy File | Target | Changes |
|---|---|---|
| `Readyaml.h` | `src/utils/readyaml.hpp` | `#pragma once`, package name → `"drone"` |
| `utils.h` (Timer) | `src/utils/timer.hpp` | Extract Timer class only, `#pragma once` |
| `utils.h` (_kbhit/_getch) | `src/utils/keyboard.hpp` | Extract keyboard fns only, `#pragma once` |
| `utils.h` (rotate_angle) | `src/utils/rotate.hpp` | Extract template only, `#pragma once` |
| `FuzzyPID.h` | `src/control/fuzzy_pid.hpp` | `#pragma once`, remove `using namespace std` |
| `FuzzyPID.cpp` | `src/control/fuzzy_pid.cpp` | Include path update |
| `AutoTune.h` | `src/control/autotune.hpp` | `#pragma once` |
| `AutoTune.cpp` | `src/control/autotune.cpp` | Include path update |
| `TrajectoryGenerator.h` | `src/trajectory/trajectory_generator.hpp` | `#pragma once` |
| `TrajectoryGenerator.cpp` | `src/trajectory/trajectory_generator.cpp` | Include path update |

[**API Surface**]

All function signatures unchanged. Only `Readyaml::getPackageName()` return
value changes from `"px4_ros_com"` to `"drone"`.

[**Constraints**]

- C-1: No dependency on `OffboardControl.h` or any ROS node class.
- C-2: FuzzyPID macros (NB/NM/NS/ZO/PS/PM/PB) preserved as-is.
- C-3: Library is internal-only — no install/export rules.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create directory structure.
2. Transfer each module with mechanical fixes only.
3. Update CMakeLists.txt with internal library target.
4. Update package.xml with new dependencies.
5. Build and verify.

[**Failure Flow**]

1. Compile errors → fix includes.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — Directories**]

```
mkdir -p src/utils src/control src/trajectory
rm src/.gitkeep
```

[**Step 2 — Transfer modules**]

For each module:
1. Copy legacy file to new location with snake_case `.hpp`/`.cpp` name.
2. Replace `#ifndef`/`#define`/`#endif` guards with `#pragma once`.
3. Remove `using namespace` from headers.
4. Fix include paths to reference new locations.
5. Fix `Readyaml` package name.
6. No other changes.

[**Step 3 — CMakeLists.txt**]

```cmake
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(drone_utils STATIC
  src/control/fuzzy_pid.cpp
  src/control/autotune.cpp
  src/trajectory/trajectory_generator.cpp
)

target_include_directories(drone_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(drone_utils yaml-cpp)
ament_target_dependencies(drone_utils Eigen3 ament_index_cpp)
```

[**Step 4 — package.xml**]

Add:
```xml
<buildtool_depend>eigen3_cmake_module</buildtool_depend>
<depend>eigen</depend>
<depend>ament_index_cpp</depend>
```

Note: `yaml-cpp` is linked directly (not an ament dep), matching legacy
behavior. No `<depend>` entry needed since it is found via system cmake.

[**Step 5 — Build**]

`colcon build --packages-select drone` — zero warnings.

## Trade-offs

- T-1: **Header extension: `.hpp` vs `.h`**
  - `.hpp`: modern ROS 2 convention (nav2, autoware, moveit2).
  - `.h`: legacy convention, less renaming churn.
  - Decision: `.hpp`. Establishes the convention early. Only new files in the
    Drone project are affected — legacy tree is untouched.

---

## Validation

[**Unit Tests**]

- V-UT-1: Deferred (NG-3).

[**Integration Tests**]

- V-IT-1: `colcon build --packages-select drone` succeeds, zero warnings.
- V-IT-2: `colcon test --packages-select drone` passes lint.
- V-IT-3: `libdrone_utils.a` exists in build output.

[**Failure / Robustness Validation**]

- V-F-1: Each `.hpp` compiles standalone.

[**Edge Case Validation**]

- V-E-1: `readyaml.hpp` `getPackageName()` returns `"drone"`.
- V-E-2: `grep -r "using namespace" src/**/*.hpp` returns zero results.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1, V-IT-3 |
| G-2 | V-IT-1 (zero warnings) |
| G-3 | All files are snake_case .hpp/.cpp |
| G-4 | V-E-1 |
| C-1 | V-F-1 (standalone compilation) |
| C-2 | FuzzyPID macros present in fuzzy_pid.hpp |
| C-3 | No install() rules for library or headers |
