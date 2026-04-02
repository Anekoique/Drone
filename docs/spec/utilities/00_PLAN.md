# `Utilities` PLAN `00`

> Status: Draft
> Feature: `utilities`
> Iteration: `00`
> Owner: Executor
> Depends on:
> - Previous Plan: `none`
> - Review: `none`
> - Master Directive: `none`

---

## Summary

Migrate all leaf-level utility modules from the legacy codebase into the
`Drone` project. These modules have zero project-local dependencies and can
be compiled and tested in isolation. This is the first phase that introduces
C++ source code into the new project.

## Log

[**Feature Introduce**]

Initial plan. Migrates 7 leaf modules from legacy `src/offboard_control/` into
a clean `src/` directory structure. Each module gets proper .h/.cpp separation,
modern C++20 style, English comments, and no `using namespace` in headers.
Dead code is removed. A compilable library target is added to CMakeLists.txt.

[**Review Adjustments**]

N/A — first iteration.

[**Master Compliance**]

N/A — first iteration.

### Changes from Previous Round

[**Added**]
Everything — initial plan.

[**Changed**]
N/A

[**Removed**]
N/A

[**Unresolved**]
N/A

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| — | — | — | First iteration, no prior findings |

---

## Spec

[**Goals**]

- G-1: Migrate `math_utils`, `readyaml`, `timer`, `keyboard`, `fuzzy_pid`,
  `autotune`, and `trajectory_generator` into `src/`.
- G-2: All migrated code compiles as a static library with C++20, `-Wall
  -Wextra -Wpedantic`, zero warnings.
- G-3: Modernize code style: no `using namespace` in headers, English
  comments, `#pragma once`, `constexpr` where possible.
- G-4: Remove dead code: `AC_WPNav.h/cpp`, `Yolo.h.bak`, `Yolo copy.bak`.
- G-5: `Readyaml` must reference package name `drone` (not `px4_ros_com`).

Non-goals:
- NG-1: No modules with project-local dependencies (math.h, PID, PosControl,
  etc.) — those belong to later features.
- NG-2: No ROS node code — these are pure C++ utilities.
- NG-3: No unit tests in this plan (deferred to a testing feature to keep
  scope focused).

[**Architecture**]

```
src/
├── utils/
│   ├── math_utils.h         (from math_utils.h)
│   ├── math_utils.cpp       (from math.cpp, extract relevant fns)
│   ├── readyaml.h           (from Readyaml.h, package name fixed)
│   ├── timer.h              (from utils.h Timer class)
│   ├── timer.cpp            (new, extract Timer impl if needed)
│   ├── keyboard.h           (from utils.h _kbhit/_getch)
│   └── rotate.h             (from utils.h rotate_angle template)
├── control/
│   ├── fuzzy_pid.h          (from FuzzyPID.h)
│   ├── fuzzy_pid.cpp        (from FuzzyPID.cpp)
│   ├── autotune.h           (from AutoTune.h)
│   └── autotune.cpp         (from AutoTune.cpp)
└── trajectory/
    ├── trajectory_generator.h   (from TrajectoryGenerator.h)
    └── trajectory_generator.cpp (from TrajectoryGenerator.cpp)
```

The directory structure groups by domain rather than having a flat dump.
`utils/` is for generic helpers, `control/` for control-theory primitives,
`trajectory/` for motion planning.

[**Invariants**]

- I-1: No header uses `using namespace` at file scope.
- I-2: All headers use `#pragma once`.
- I-3: Every header is self-contained (compiles standalone).
- I-4: No module includes another project-local module outside its own
  directory, except `readyaml.h` which is a shared utility.
- I-5: `Readyaml::getPackageName()` returns `"drone"`.

[**Data Structure**]

Module migration map:

| Legacy File | Target File(s) | Changes |
|---|---|---|
| `math_utils.h` | `src/utils/math_utils.h` | `#pragma once`, English comments |
| `math.cpp` (is_zero, is_positive, is_negative) | `src/utils/math_utils.cpp` | Extract only math_utils fns |
| `Readyaml.h` | `src/utils/readyaml.h` | Fix package name, English comments |
| `utils.h` (Timer class) | `src/utils/timer.h` | Extract Timer only |
| `utils.h` (_kbhit, _getch) | `src/utils/keyboard.h` | Extract keyboard only |
| `utils.h` (rotate_angle) | `src/utils/rotate.h` | Extract rotation template only |
| `FuzzyPID.h/cpp` | `src/control/fuzzy_pid.h/cpp` | Remove `using namespace std`, English comments |
| `AutoTune.h/cpp` | `src/control/autotune.h/cpp` | English comments |
| `TrajectoryGenerator.h/cpp` | `src/trajectory/trajectory_generator.h/cpp` | English comments |
| `AC_WPNav.h/cpp` | (deleted) | Dead code, all commented out |
| `Yolo.h.bak`, `Yolo copy.bak` | (deleted) | Backup files |

[**API Surface**]

No public API changes. All functions and classes retain their existing
signatures. The only semantic change is `Readyaml::getPackageName()` returning
`"drone"` instead of `"px4_ros_com"`.

[**Constraints**]

- C-1: Migrated code must compile in isolation — no dependency on
  `OffboardControl.h` or any ROS node class.
- C-2: `FuzzyPID` `#define` macros (NB, NM, NS, ZO, PS, PM, PB) must be
  preserved as-is for now — they are used in rule table initialization
  throughout the codebase. Replacing them with `constexpr` is deferred.
- C-3: `readyaml.h` config path resolution must work with ament install
  layout (`share/drone/config/`).

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create directory structure (`src/utils/`, `src/control/`, `src/trajectory/`).
2. Migrate each module: copy, clean, modernize.
3. Update `CMakeLists.txt`: add `find_package` deps, create library target.
4. Update `package.xml`: add build/runtime dependencies.
5. Build and verify zero warnings.

[**Failure Flow**]

1. Compile errors → fix includes, missing deps.
2. Warnings → fix code until `-Wpedantic` clean.

[**State Transition**]

Not applicable — no runtime state.

### Implementation Plan

[**Step 1 — Directory structure**]

```
mkdir -p src/utils src/control src/trajectory
```
Remove `src/.gitkeep`.

[**Step 2 — Migrate utils/**]

- `math_utils.h/cpp`: minimal — 3 function declarations + implementations.
- `readyaml.h`: fix `getPackageName()` → `"drone"`, clean path resolution,
  use `std::filesystem::path` properly.
- `timer.h`: extract `Timer` class from `utils.h`. Header-only.
- `keyboard.h`: extract `_kbhit()`/`_getch()` from `utils.h`. Header-only.
- `rotate.h`: extract `rotate_angle()` template from `utils.h`. Header-only.

[**Step 3 — Migrate control/**]

- `fuzzy_pid.h/cpp`: remove `using namespace std`, translate comments.
- `autotune.h/cpp`: translate Chinese comments to English.

[**Step 4 — Migrate trajectory/**]

- `trajectory_generator.h/cpp`: translate comments, clean includes.

[**Step 5 — Update CMakeLists.txt**]

```cmake
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(drone_utils STATIC
  src/utils/math_utils.cpp
  src/control/fuzzy_pid.cpp
  src/control/autotune.cpp
  src/trajectory/trajectory_generator.cpp
)

target_include_directories(drone_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(drone_utils Eigen3 yaml-cpp ament_index_cpp)
```

[**Step 6 — Update package.xml**]

Add:
```xml
<depend>eigen</depend>
<depend>yaml-cpp</depend>
<depend>ament_index_cpp</depend>
<buildtool_depend>eigen3_cmake_module</buildtool_depend>
```

[**Step 7 — Build + verify**]

`colcon build --packages-select drone` with zero warnings.

## Trade-offs

- T-1: **Static vs shared library**
  - Static: simpler linking, no symbol visibility concerns, single binary.
  - Shared: faster incremental linking, can be reused by other packages.
  - Decision: Static. This is an internal library, not exported. Single
    deployment binary is simpler for Raspberry Pi.

- T-2: **Directory structure: flat vs domain-grouped**
  - Flat `src/`: simple, all files in one place.
  - Domain-grouped `src/utils/`, `src/control/`, `src/trajectory/`: clearer
    boundaries, matches the target architecture in ROADMAP.md.
  - Decision: Domain-grouped. Matches the final architecture and makes
    dependency direction visible in the directory tree.

- T-3: **Keep FuzzyPID macros (NB/NM/NS/ZO/PS/PM/PB) vs replace with constexpr**
  - Macros: ugly but used in C-style array initializers throughout codebase.
  - constexpr: cleaner but requires updating all call sites.
  - Decision: Keep macros for now (C-2). Replace when the consuming code
    (PosControl) is migrated and can be updated atomically.

---

## Validation

[**Unit Tests**]

- V-UT-1: Deferred (NG-3).

[**Integration Tests**]

- V-IT-1: `colcon build --packages-select drone` succeeds with zero warnings.
- V-IT-2: `colcon test --packages-select drone` passes lint.
- V-IT-3: Library `libdrone_utils.a` exists in build output.

[**Failure / Robustness Validation**]

- V-F-1: Each header compiles standalone (include it in an empty .cpp and
  compile — no missing deps).

[**Edge Case Validation**]

- V-E-1: `readyaml.h` `getPackageName()` returns `"drone"`.
- V-E-2: No `using namespace` at file scope in any header (`grep` check).

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1 (all modules compile) |
| G-2 | V-IT-1 (zero warnings), V-IT-3 (library exists) |
| G-3 | V-E-2 (no using namespace), V-F-1 (self-contained headers) |
| G-4 | Verify AC_WPNav, .bak files not present |
| G-5 | V-E-1 |
| C-1 | V-F-1 |
| C-2 | FuzzyPID macros preserved (code review) |
| C-3 | V-E-1, readyaml resolves config path correctly |
