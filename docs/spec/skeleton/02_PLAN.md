# `Skeleton` PLAN `02`

> Status: Draft
> Feature: `skeleton`
> Iteration: `02`
> Owner: Executor
> Depends on:
> - Previous Plan: `01_PLAN.md`
> - Review: `01_REVIEW.md`
> - Master Directive: `01_MASTER.md`

---

## Summary

Final revision of the build skeleton. Fixes the package.xml export stanza,
adopts an ament-aligned `.clang-format`, upgrades to C++20, renames config
files to drop `_config` suffixes, and adds install-layout validation.

## Log

[**Feature Introduce**]

- `.clang-format` sourced from ament/autoware conventions (Google base,
  `ColumnLimit: 100`, `PointerAlignment: Middle`, custom brace wrapping,
  include sorting categories).
- C++20 standard (GCC 11 on Ubuntu 22.04 supports it fully; enables
  `std::ranges`, concepts, designated initializers, `std::format`).
- Config files renamed: drop `_config` suffix, keep names descriptive.

[**Review Adjustments**]

- R-001 (HIGH): Added `<export><build_type>ament_cmake</build_type></export>`
  to package.xml spec.
- R-002 (MEDIUM): Added install-layout validation step checking
  `install/drone/share/drone/config/` contents.
- R-003 (LOW): Rephrased log language — "no runtime or future-phase deps"
  instead of "no deps at all".

[**Master Compliance**]

- M-001: Project layout follows modern ROS 2 conventions (ament_cmake,
  config installed to `share/<pkg>/config`, standard directory structure).
- M-002: Upgraded from C++17 to C++20. Ubuntu 22.04 ships GCC 11 which
  has full C++20 support. Enables concepts, ranges, designated initializers.
- M-003: `.clang-format` derived from ament official + autoware conventions.
  Google base, 100-col limit, custom brace wrapping, include sorting.
- M-004: Config files renamed — `pos_config.yaml` → `position.yaml`,
  `can_config.yaml` → `airdrop.yaml`, `land_config.yaml` → `landing.yaml`.

### Changes from Previous Round

[**Added**]
- `<export>` stanza in package.xml.
- Install-layout validation step.
- Include sorting categories in `.clang-format`.

[**Changed**]
- C++ standard: C++17 → C++20.
- `.clang-format`: custom Google+4-space → ament-standard Google+2-space.
- Config filenames: legacy names → clean names without `_config`.

[**Removed**]
Nothing.

[**Unresolved**]
Nothing.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | `<export>` stanza added to package.xml spec |
| Review | R-002 | Accepted | V-IT-3 added: verify installed config layout |
| Review | R-003 | Accepted | Log language rephrased |
| Master | M-001 | Applied | Standard ament_cmake layout, config in `share/` |
| Master | M-002 | Applied | C++20 standard |
| Master | M-003 | Applied | `.clang-format` from ament/autoware conventions |
| Master | M-004 | Applied | Config files renamed, `_config` suffix removed |

---

## Spec

[**Goals**]

- G-1: Create `CMakeLists.txt` and `package.xml` that build as an empty
  ament_cmake package named `drone` with C++20.
- G-2: Copy and rename config YAML files from legacy.
- G-3: Add `.clang-format` based on ament/autoware conventions.
- G-4: `colcon build --packages-select drone` succeeds.

Non-goals:
- NG-1: No C++ source code.
- NG-2: No launch files.
- NG-3: No dependency pre-loading for future phases.

[**Architecture**]

```
Drone/
├── CMakeLists.txt
├── package.xml
├── .clang-format
├── config/
│   ├── mission.yaml      (from OffboardControl.yaml)
│   ├── camera.yaml       (from camera.yaml)
│   ├── position.yaml     (from pos_config.yaml)
│   ├── airdrop.yaml      (from can_config.yaml)
│   └── landing.yaml      (from land_config.yaml)
├── docs/  (existing)
├── scripts/  (existing)
└── src/   (empty, .gitkeep)
```

[**Invariants**]

- I-1: Package name is `drone` in CMakeLists.txt and package.xml.
- I-2: Config YAML field names and values are identical to legacy — only
  filenames change.
- I-3: No compile targets — CMakeLists.txt only installs config and declares
  the ament package.
- I-4: `.clang-format` is compatible with `ament_clang_format` checks.

[**Data Structure**]

Config file renames:

| Legacy | New | Change |
|---|---|---|
| `OffboardControl.yaml` | `mission.yaml` | Clarify scope |
| `camera.yaml` | `camera.yaml` | Unchanged |
| `pos_config.yaml` | `position.yaml` | Drop `_config`, drop `pos_` abbreviation |
| `can_config.yaml` | `airdrop.yaml` | Rename to match function |
| `land_config.yaml` | `landing.yaml` | Drop `_config` |

[**API Surface**]

None — no executables, no launch files, no ROS interfaces.

[**Constraints**]

- C-1: `.clang-format` must be based on ament conventions (Google base).
- C-2: Config YAML values must not change — only filenames.
- C-3: package.xml must include `<export><build_type>ament_cmake</build_type></export>`.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create `CMakeLists.txt`.
2. Create `package.xml`.
3. Copy and rename config files.
4. Create `.clang-format`.
5. Create `src/.gitkeep`.
6. Validate build and install layout.

[**Failure Flow**]

1. `colcon build` fails → fix package metadata.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — CMakeLists.txt**]

```cmake
cmake_minimum_required(VERSION 3.14)
project(drone VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

[**Step 2 — package.xml**]

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drone</name>
  <version>0.1.0</version>
  <description>Autonomous multirotor UAV mission system</description>
  <maintainer email="23050820@hdu.edu.cn">drone</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

[**Step 3 — Config files**]

Copy with rename. Content unchanged:
```
cp config/OffboardControl.yaml → config/mission.yaml
cp config/camera.yaml          → config/camera.yaml
cp config/pos_config.yaml      → config/position.yaml
cp config/can_config.yaml      → config/airdrop.yaml
cp config/land_config.yaml     → config/landing.yaml
```

[**Step 4 — .clang-format**]

Based on ament official + autoware include categories:

```yaml
Language: Cpp
BasedOnStyle: Google

AccessModifierOffset: -2
AlignAfterOpenBracket: AlwaysBreak
AllowShortFunctionsOnASingleLine: InlineOnly
BraceWrapping:
  AfterClass: true
  AfterEnum: true
  AfterFunction: true
  AfterNamespace: true
  AfterStruct: true
BreakBeforeBraces: Custom
ColumnLimit: 100
ConstructorInitializerIndentWidth: 0
ContinuationIndentWidth: 2
DerivePointerAlignment: false
PointerAlignment: Middle
ReflowComments: false
SortIncludes: true
IncludeCategories:
  - Regex: <[a-z_]*>
    Priority: 6
    CaseSensitive: true
  - Regex: <.*\.h>
    Priority: 5
    CaseSensitive: true
  - Regex: .*_msgs/.*
    Priority: 3
    CaseSensitive: true
  - Regex: .*_srvs/.*
    Priority: 3
    CaseSensitive: true
  - Regex: <.*>
    Priority: 2
    CaseSensitive: true
  - Regex: '".*"'
    Priority: 1
    CaseSensitive: true
```

[**Step 5 — Validate**]

`colcon build` + `colcon test` + install layout check.

## Trade-offs

- T-1: **C++20 vs C++17**
  - C++20: concepts, ranges, designated initializers, `std::format`.
    GCC 11 (Ubuntu 22.04) has full C++20 core language and most library support.
  - C++17: safer if targeting older toolchains.
  - Decision: C++20. The project targets Ubuntu 22.04 exclusively. If a
    specific C++20 library feature is missing on the Pi's GCC, it can be
    worked around per-feature — the standard flag itself is safe.

- T-2: **ament .clang-format (2-space) vs custom 4-space**
  - Ament standard: 2-space indent, compatible with `ament_clang_format`.
  - Custom 4-space: matches legacy code, less reformatting.
  - Decision: Ament standard. Legacy code will be reformatted during
    migration anyway. Staying ament-compatible avoids fighting the linter.

---

## Validation

[**Unit Tests**]
- V-UT-1: Not applicable — no code.

[**Integration Tests**]
- V-IT-1: `colcon build --packages-select drone` succeeds.
- V-IT-2: `colcon test --packages-select drone` passes lint.
- V-IT-3: Installed layout contains all 5 config files:
  `install/drone/share/drone/config/{mission,camera,position,airdrop,landing}.yaml`

[**Failure / Robustness Validation**]
- V-F-1: Package builds with only `ament_cmake` installed.

[**Edge Case Validation**]
- V-E-1: Config YAML values are byte-identical to legacy (`diff` check).

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1, V-IT-2 |
| G-2 | V-E-1, V-IT-3 |
| G-3 | `.clang-format` exists, `BasedOnStyle: Google`, ament-compatible |
| G-4 | V-IT-1 |
| C-1 | `.clang-format` matches ament conventions |
| C-2 | V-E-1 |
| C-3 | V-IT-2 (lint checks package.xml structure) |
