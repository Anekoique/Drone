# `Skeleton` PLAN `00`

> Status: Draft
> Feature: `skeleton`
> Iteration: `00`
> Owner: Executor
> Depends on:
> - Previous Plan: `none`
> - Review: `none`
> - Master Directive: `none`

---

## Summary

Establish the build system, configuration files, launch infrastructure, and
code formatting rules for the new `Drone` project. After this phase, the
project can be placed in a colcon workspace, built (producing zero executables
for now), and launched as an empty ROS 2 package. This is the foundation for
all subsequent code migration phases.

## Log

[**Feature Introduce**]

This is the initial plan. It creates the ament_cmake build skeleton, migrates
configuration YAMLs, sets up launch files with parameterized arguments, and
enforces a `.clang-format` for all future C++ code.

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

- G-1: Create a valid `CMakeLists.txt` and `package.xml` that build as an
  empty ament_cmake package named `drone`.
- G-2: Migrate all YAML config files from the legacy project with clear naming
  and no behavioral changes.
- G-3: Create a parameterized ROS 2 launch file that loads configs and
  supports `sim_mode` / `debug_mode` / `print_info` / `fast_mode` arguments.
- G-4: Enforce consistent C++ formatting via `.clang-format`.
- G-5: The package must pass `colcon build` and `colcon test` (lint only)
  on Ubuntu 22.04 with ROS 2 Humble.

Non-goals:
- NG-1: No C++ source code is migrated in this phase.
- NG-2: No runtime functionality — the package is a build/config skeleton only.

[**Architecture**]

```
Drone/
├── CMakeLists.txt
├── package.xml
├── .clang-format
├── config/
│   ├── mission.yaml          (from OffboardControl.yaml)
│   ├── camera.yaml           (from camera.yaml, unchanged)
│   ├── position_control.yaml (from pos_config.yaml)
│   ├── airdrop.yaml          (from can_config.yaml)
│   └── landing.yaml          (from land_config.yaml)
├── launch/
│   └── drone.launch.py
├── docs/
│   └── (existing docs)
├── scripts/
│   └── deps/ (existing)
└── src/                      (empty, created for future phases)
```

[**Invariants**]

- I-1: The package name is `drone` everywhere (CMakeLists, package.xml, launch).
- I-2: Config files are pure data — no logic, no code generation.
- I-3: Config file contents are value-identical to legacy (field names and
  values preserved); only file names change for clarity.
- I-4: The launch file must be usable with `ros2 launch drone drone.launch.py`.
- I-5: `.clang-format` applies to all `.h`, `.cpp` files under `src/`.

[**Data Structure**]

Config file renames:

| Legacy | New | Rationale |
|---|---|---|
| `config/OffboardControl.yaml` | `config/mission.yaml` | Contains heading, waypoint offsets, servo — mission-level params |
| `config/camera.yaml` | `config/camera.yaml` | Name already clear |
| `config/pos_config.yaml` | `config/position_control.yaml` | PID gains for position controller |
| `config/can_config.yaml` | `config/airdrop.yaml` | PID + servo params for airdrop targeting |
| `config/land_config.yaml` | `config/landing.yaml` | PID + limits for landing approach |

[**API Surface**]

Launch arguments:

```python
# drone.launch.py
DeclareLaunchArgument('sim_mode',    default_value='false')
DeclareLaunchArgument('debug_mode',  default_value='false')
DeclareLaunchArgument('print_info',  default_value='false')
DeclareLaunchArgument('fast_mode',   default_value='false')
DeclareLaunchArgument('mavros_ns',   default_value='/mavros/')
```

These map 1:1 to the legacy `OffboardControl_Base` ROS parameters.

[**Constraints**]

- C-1: CMakeLists.txt must list all `find_package` dependencies from the
  legacy build so that `rosdep` can resolve them, even though no targets
  compile yet. This prevents surprises in Phase 1.
- C-2: The `package.xml` dependency list must match CMakeLists.txt exactly.
- C-3: `.clang-format` must be based on a well-known style (Google or LLVM)
  with minimal customization to reduce bikeshedding.
- C-4: Config file field names and values must not change — only filenames.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create `CMakeLists.txt` with project name `drone`, C++17, all
   `find_package` deps, but no `add_executable` targets.
2. Create `package.xml` format 3 with matching `<depend>` entries.
3. Copy and rename config YAML files from legacy.
4. Create `launch/drone.launch.py` with declared arguments and config loading.
5. Create `.clang-format` based on Google style.
6. Create empty `src/` directory with a `.gitkeep`.
7. Validate: `colcon build --packages-select drone`.

[**Failure Flow**]

1. If `colcon build` fails on missing deps → fix `package.xml`/CMakeLists.
2. If lint tests fail → fix package metadata (license, maintainer, etc.).

[**State Transition**]

Not applicable — no runtime state in this phase.

### Implementation Plan

[**Step 1 — CMakeLists.txt + package.xml**]

Create the build system files. CMakeLists.txt declares all dependencies
(commented targets placeholder). package.xml lists build/exec/test deps.
Key changes from legacy:
- Package name: `px4_ros_com` → `drone`
- C++ standard: C++14 → C++17
- Remove `file(GLOB ...)` — future phases will list sources explicitly
- Remove commented-out Python install blocks

[**Step 2 — Config migration**]

Copy the 5 YAML files with new names. No content changes. Add a comment
header to each file noting the legacy filename for traceability.

[**Step 3 — Launch file**]

Python-based launch file that:
- Declares 5 launch arguments (sim_mode, debug_mode, print_info, fast_mode, mavros_ns)
- Will eventually launch the `drone` node with parameters (placeholder comment)
- Loads config files via `ament_index_python` path resolution

[**Step 4 — .clang-format**]

Based on Google style with these overrides:
- `ColumnLimit: 120` (wider for ROS code)
- `IndentWidth: 4` (match legacy indentation)
- `PointerAlignment: Left` (`float* ptr` not `float *ptr`)

[**Step 5 — Validation**]

Run `colcon build` and `colcon test` to verify the skeleton.

## Trade-offs

- T-1: **C++ standard: C++17 vs C++14**
  - Legacy uses C++14. C++17 adds `std::optional`, `std::string_view`,
    structured bindings, `if constexpr` — all useful for the refactoring.
  - Adv: Cleaner code in future phases, `std::optional` already used in
    legacy `CameraGimbal.h`.
  - Disadv: Raspberry Pi cross-compilation toolchains must support C++17
    (GCC 9+ does, Ubuntu 22.04 ships GCC 11).
  - Recommendation: Use C++17.

- T-2: **Config file renaming vs keeping legacy names**
  - Option A: Rename for clarity (`can_config.yaml` → `airdrop.yaml`).
  - Option B: Keep legacy names to avoid confusion during migration.
  - Adv A: Cleaner, self-documenting. Adv B: Zero friction.
  - Recommendation: Rename, but add a comment header with the legacy name.

- T-3: **clang-format style: Google vs LLVM vs custom**
  - Google: widely used in ROS ecosystem, 2-space indent.
  - LLVM: similar but different brace style.
  - Custom with 4-space indent: matches legacy code, less reformatting churn.
  - Recommendation: Google base + 4-space indent override for minimal churn.

---

## Validation

[**Unit Tests**]

- V-UT-1: Not applicable — no code to unit-test in this phase.

[**Integration Tests**]

- V-IT-1: `colcon build --packages-select drone` succeeds with zero errors.
- V-IT-2: `colcon test --packages-select drone` passes all lint checks.
- V-IT-3: `ros2 launch drone drone.launch.py` starts and exits cleanly
  (no node to run, but launch file must parse without errors).

[**Failure / Robustness Validation**]

- V-F-1: Building without MAVROS installed reports a clear cmake error
  (not a cryptic linker failure).
- V-F-2: Missing config files at launch time produce a descriptive error.

[**Edge Case Validation**]

- V-E-1: Config YAML values round-trip correctly (no float precision loss
  from copy).
- V-E-2: Launch file handles all boolean argument values (`true`/`false`,
  `True`/`False`, `1`/`0`).

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1, V-IT-2 |
| G-2 | V-E-1 |
| G-3 | V-IT-3, V-E-2 |
| G-4 | `.clang-format` exists, `clang-format --dry-run` on empty src/ passes |
| G-5 | V-IT-1, V-IT-2 |
| C-1 | V-F-1 |
| C-2 | V-IT-2 (ament_lint checks dep consistency) |
| C-3 | `.clang-format` BasedOnStyle field set |
| C-4 | V-E-1, diff of YAML values against legacy |
