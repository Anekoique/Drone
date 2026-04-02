# `Skeleton` PLAN `01`

> Status: Draft
> Feature: `skeleton`
> Iteration: `01`
> Owner: Executor
> Depends on:
> - Previous Plan: `00_PLAN.md`
> - Review: `00_REVIEW.md`
> - Master Directive: `00_MASTER.md`

---

## Summary

Minimal build skeleton for the `drone` ament_cmake package: CMakeLists.txt,
package.xml, config files, and .clang-format. No launch file, no future-phase
dependencies, no dead infrastructure.

## Log

[**Feature Introduce**]

Stripped the plan to its core: build system + config + formatting. Removed
launch file, removed speculative dependency pre-loading, removed config
renaming.

[**Review Adjustments**]

- R-001 (HIGH): Dependency source of truth fixed. `package.xml` declares only
  what this phase needs. CMakeLists.txt has no `find_package` calls — there are
  no compile targets.
- R-002 (HIGH): Resolved by removing the launch file entirely.
- R-003 (MEDIUM): Resolved — with no compile targets, there is no dep parity
  to verify. `package.xml` lists only `ament_cmake` build tool dep.

[**Master Compliance**]

- M-001: Launch directory and file removed. The legacy launch files are unused
  dead code; the project runs via `ros2 run` directly.
- M-002: Removed speculative future dependencies from CMakeLists.txt. Removed
  config file renaming (premature without runtime code). Removed launch
  infrastructure and all its dependencies.

### Changes from Previous Round

[**Added**]
- Explicit justification for C++17 (legacy already uses `std::optional`).

[**Changed**]
- CMakeLists.txt scope: from "all legacy deps" to "build-tool-only, no targets".
- package.xml scope: from "mirror all CMake deps" to "ament_cmake only".
- Config strategy: keep legacy filenames as-is, defer renaming to code migration.

[**Removed**]
- G-3 (launch file) — dead infrastructure, not used in actual workflow.
- C-1 (pre-load all future deps) — couples skeleton to full migration closure.
- C-2 (package.xml must match CMakeLists exactly) — no targets to match against.
- T-2 (config renaming trade-off) — deferred.
- All launch-related validation (V-IT-3, V-E-2, V-F-2).

[**Unresolved**]
None.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | CMakeLists has no `find_package`; `package.xml` lists only `ament_cmake` |
| Review | R-002 | Accepted | Launch file removed from scope |
| Review | R-003 | Accepted | No dep parity needed — no compile targets |
| Master | M-001 | Applied | Launch directory removed |
| Master | M-002 | Applied | Removed speculative deps, config renaming, launch infra |
| Review | TR-1 | Accepted | Config files keep legacy names |
| Review | TR-2 | Accepted | C++17 kept; justified by existing `std::optional` usage |

---

## Spec

[**Goals**]

- G-1: Create `CMakeLists.txt` and `package.xml` that build as an empty
  ament_cmake package named `drone` with C++17.
- G-2: Copy config YAML files from legacy with no changes.
- G-3: Add `.clang-format` for all future C++ code.
- G-4: `colcon build --packages-select drone` succeeds.

Non-goals:
- NG-1: No C++ source code.
- NG-2: No launch files.
- NG-3: No config renaming (deferred to code migration phase).
- NG-4: No dependency pre-loading for future phases.

[**Architecture**]

```
Drone/
├── CMakeLists.txt
├── package.xml
├── .clang-format
├── config/
│   ├── OffboardControl.yaml
│   ├── camera.yaml
│   ├── pos_config.yaml
│   ├── can_config.yaml
│   └── land_config.yaml
├── docs/  (existing)
├── scripts/  (existing)
└── src/   (empty, .gitkeep)
```

[**Invariants**]

- I-1: Package name is `drone` in CMakeLists.txt and package.xml.
- I-2: Config files are byte-identical copies of legacy originals.
- I-3: No compile targets — `CMakeLists.txt` only sets up the ament package.

[**Data Structure**]

Config files copied verbatim:

| Legacy Path | Drone Path |
|---|---|
| `config/OffboardControl.yaml` | `config/OffboardControl.yaml` |
| `config/camera.yaml` | `config/camera.yaml` |
| `config/pos_config.yaml` | `config/pos_config.yaml` |
| `config/can_config.yaml` | `config/can_config.yaml` |
| `config/land_config.yaml` | `config/land_config.yaml` |

[**API Surface**]

None — no executables, no launch files, no ROS interfaces.

[**Constraints**]

- C-1: `.clang-format` must be based on a well-known style with minimal
  customization.
- C-2: Config file contents must not change — byte-identical to legacy.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create `CMakeLists.txt` (project `drone`, C++17, ament_cmake, install config).
2. Create `package.xml` (format 3, `ament_cmake` buildtool dep only).
3. Copy 5 config YAML files from legacy.
4. Create `.clang-format`.
5. Create `src/.gitkeep`.
6. Validate: `colcon build --packages-select drone`.

[**Failure Flow**]

1. `colcon build` fails → fix package metadata (license, maintainer, etc.).

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — CMakeLists.txt + package.xml**]

CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 3.14)
project(drone VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

package.xml: format 3, name `drone`, `ament_cmake` buildtool dep,
`ament_lint_auto` + `ament_lint_common` test deps.

[**Step 2 — Config files**]

`cp` the 5 YAML files. No edits.

[**Step 3 — .clang-format**]

Google base with overrides:
- `ColumnLimit: 120`
- `IndentWidth: 4`
- `PointerAlignment: Left`

C++17 already used in legacy (`std::optional` in CameraGimbal.h), and
Ubuntu 22.04 ships GCC 11 which fully supports it.

[**Step 4 — Validate**]

`colcon build --packages-select drone` and `colcon test --packages-select drone`.

## Trade-offs

- T-1: **C++17 vs C++14**
  - Keep C++17. Legacy already uses `std::optional` (CameraGimbal.h).
    Ubuntu 22.04 ships GCC 11. No compatibility risk.

- T-2: **clang-format style**
  - Google base + 4-space indent. Matches legacy indentation, minimizes
    reformatting churn when code is migrated.

---

## Validation

[**Unit Tests**]
- V-UT-1: Not applicable — no code.

[**Integration Tests**]
- V-IT-1: `colcon build --packages-select drone` succeeds.
- V-IT-2: `colcon test --packages-select drone` passes lint.

[**Failure / Robustness Validation**]
- V-F-1: Package builds with only `ament_cmake` installed (no MAVROS needed).

[**Edge Case Validation**]
- V-E-1: Config YAML files are byte-identical to legacy (`diff` check).

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1, V-IT-2 |
| G-2 | V-E-1 |
| G-3 | `.clang-format` exists with `BasedOnStyle: Google` |
| G-4 | V-IT-1 |
| C-1 | `.clang-format` `BasedOnStyle` field set |
| C-2 | V-E-1 |
