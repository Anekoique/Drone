# `Math & PID` PLAN `01`

> Status: Draft
> Feature: `math-pid`
> Iteration: `01`
> Owner: Executor
> Depends on:
> - Previous Plan: `00_PLAN.md`
> - Review: `00_REVIEW.md`
> - Master Directive: `00_MASTER.md`

---

## Summary

Transfer math helpers, PID, BasicPID, and SCurve with clean naming.
Strictly mechanical transfer — no algorithm changes, no config-path
migration, no merging of module boundaries.

## Log

[**Feature Introduce**]

Clean naming scheme: files follow `snake_case.hpp` convention, classes
use `PascalCase`, everything in `drone::` namespace. Module boundaries
match the legacy structure to keep migration mechanical.

[**Review Adjustments**]

- R-001 (HIGH): Accepted. Roadmap module boundaries preserved.
  `math_utils` and `math` remain as two separate headers. BasicPID keeps
  its own file. Roadmap updated to reflect final file names.
- R-002 (HIGH): Accepted. BasicPID full API surface documented and preserved:
  `load_params()`, `read_goal()`, `compute()`, `update()`, and public
  velocity fields.
- R-003 (MEDIUM): Noted. Config-path migration removed from this phase.
  BasicPID and PID still use legacy `Readyaml` include path internally —
  they already depend on `drone/utils/readyaml.hpp` which was migrated
  in Phase 1. No new config-loading behavior introduced.
- R-004 (MEDIUM): Accepted. `safe_sqrt` is fully inline in the header.
  No explicit instantiations in .cpp.

[**Master Compliance**]

- M-001: Naming cleaned up with a consistent scheme:

  | Legacy | Header | Class | Rationale |
  |---|---|---|---|
  | `math_utils.h` | `math_utils.hpp` | free functions | Predicates: is_zero, is_positive, etc. |
  | `math.h` | `math_types.hpp` | free functions + typedefs | Eigen helpers: sq, norm, rotate_xy, etc. |
  | `PID.h` | `pid.hpp` | `PID` | Standard PID controller (kept — well-known acronym) |
  | `MYPID.h` | `basic_pid.hpp` | `BasicPID` | Simple 3-axis PID wrapper (descriptive rename) |
  | `SCurve.h` | `scurve.hpp` | `SCurve` | S-curve waypoint path planner (kept) |

  File names: all `snake_case.hpp`. Class names: all `PascalCase`.
  No ambiguous abbreviations, no `_t` suffixes, no ALL_CAPS class names.

### Changes from Previous Round

[**Added**]
- Full BasicPID API surface documentation.
- Explicit `safe_sqrt` strategy (inline header-only).
- `math_types.hpp` as separate header (was merged in round 00).

[**Changed**]
- `BasicPID` class → `BasicPID` (PascalCase, M-001) but file stays `basic_pid.hpp` (R-001).
- `math_utils` + `math` kept as two headers (R-001, TR-2).
- Removed ConfigLoader migration from BasicPID scope (R-003).

[**Removed**]
- `SimplePID` rename (TR-1, R-001).
- math module merge (TR-2).
- Config-path runtime validation (R-003).

[**Unresolved**]
Nothing.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | Roadmap boundaries preserved, two math headers |
| Review | R-002 | Accepted | Full BasicPID API surface listed in spec |
| Review | R-003 | Accepted | Config-path migration removed from scope |
| Review | R-004 | Accepted | safe_sqrt fully inline |
| Review | TR-1 | Partially accepted | File `basic_pid.hpp` kept, class renamed `BasicPID` per M-001 |
| Review | TR-2 | Accepted | math_utils + math_types kept separate |
| Master | M-001 | Applied | Consistent snake_case files, PascalCase classes |

---

## Spec

[**Goals**]

- G-1: Transfer `math_utils`, `math`, `PID`, `BasicPID`, `SCurve` into
  `include/drone/math/` and `src/math/`.
- G-2: All code compiles in `drone_utils`, C++20, zero warnings.
- G-3: Remove `using namespace Eigen` from all headers.
- G-4: Replace `MIN`/`MAX`/`#undef` macros with `std::min`/`std::max`.
- G-5: Consolidate duplicated `is_zero`/`constrain_float` into `math_utils`.
- G-6: Clean naming per M-001.

Non-goals:
- NG-1: No algorithm changes.
- NG-2: No unit tests.
- NG-3: No config-path migration.

[**Architecture**]

```
include/drone/math/
├── math_utils.hpp    predicates, constrain_float, safe_sqrt
├── math_types.hpp    Eigen helpers: sq, norm, rotate_xy, kinematic_limit, is_equal
├── pid.hpp           PID controller
├── basic_pid.hpp         BasicPID (3-axis simple PID wrapper)
└── scurve.hpp        SCurve path planner

src/math/
├── math_utils.cpp    is_equal(Vector4f), constrain_float, kinematic_limit
├── pid.cpp           PID implementation
└── scurve.cpp        SCurve implementation
```

[**Invariants**]

- I-1: No `using namespace Eigen` in any header.
- I-2: All headers `#pragma once`, self-contained.
- I-3: No duplicated function definitions across .cpp files.
- I-4: No `#undef MIN`/`#undef MAX`.
- I-5: No behavioral changes.
- I-6: `isfinite` macro removed — use `std::isfinite`.

[**Data Structure**]

BasicPID preserved API surface:

```cpp
namespace drone {
class BasicPID {
public:
  BasicPID() = default;
  // Legacy method names preserved for mechanical migration
  void readPIDParameters(const std::string& filename, const std::string& pid_name);
  float read_goal(const std::string& filename, const std::string& goal_name);
  void Mypid(float x_target, float y_target, float z_target,
             float x_now, float y_now, float z_now, float dt);
  float compute(float setpoint, float measured, float dt);

  float velocity_x = 0, velocity_y = 0, velocity_z = 0;
  float kp_ = 0, ki_ = 0, kd_ = 0;
  float output_limit_ = 0;
  float integral_limit = 0;
};
}
```

[**API Surface**]

All legacy function signatures preserved within `drone::` namespace.
Eigen types referenced via `Eigen::Vector3f` etc. in headers.

[**Constraints**]

- C-1: SCurve uses `Eigen::Vector3f` throughout.
- C-2: PID debug macros preserved.
- C-3: PID::Defaults struct and YAML loading preserved.
- C-4: `math_types.hpp` includes `<Eigen/Core>` and `<Eigen/Geometry>`.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create directories.
2. Transfer math_utils (predicates + constrain_float + safe_sqrt).
3. Transfer math_types (Eigen helpers).
4. Transfer PID (remove duplicated math fns).
5. Transfer BasicPID (rename class only).
6. Transfer SCurve.
7. Update CMakeLists.txt.
8. Format + build + test via Docker.

[**Failure Flow**]

1. Compile errors → fix `Eigen::` prefix, missing includes.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — math_utils.hpp/cpp**]

Header: `is_zero`, `is_positive`, `is_negative`, `constrain_float` declarations.
`safe_sqrt` as inline template. No Eigen dependency.

Source: implementations of the above non-template functions.

[**Step 2 — math_types.hpp + math_utils.cpp additions**]

Header: includes `<Eigen/Core>`, `<Eigen/Geometry>`. Templates: `sq`, `norm`,
`rotate_xy`, `is_equal<T>`. Declarations: `is_equal(Eigen::Vector4f, ...)`,
`kinematic_limit(...)`.

Source additions: `is_equal(Vector4f)`, `kinematic_limit` in math_utils.cpp.

[**Step 3 — pid.hpp/cpp**]

Transfer PID class into `drone::` namespace. Remove `#ifndef MATH_H` block
with duplicated helpers — include `drone/math/math_utils.hpp` instead.
Remove trailing `#ifndef POSCONTROL_H` defaults block. Remove commented-out
`main()`.

[**Step 4 — basic_pid.hpp**]

Header-only. Class renamed `MYPID` → `BasicPID`. All legacy method names
preserved (`readPIDParameters`, `read_goal`, `Mypid`, `compute`).
Uses `drone::ConfigLoader::load()` (Phase 1 migrated loader).
`std::clamp` replaces manual limit checks.

[**Step 5 — scurve.hpp/cpp**]

Replace `Vector3f` → `Eigen::Vector3f`. Remove legacy `#include "math.h"` →
`#include "drone/math/math_types.hpp"`.

[**Step 6 — CMakeLists.txt**]

Add `src/math/math_utils.cpp`, `src/math/pid.cpp`, `src/math/scurve.cpp`
to `drone_utils` sources.

[**Step 7 — Format + Docker build**]

Format all files with clang-format, build and test via `./scripts/build.sh`.

## Trade-offs

- T-1: **`BasicPID` vs legacy `MYPID`**
  - `BasicPID`: descriptive — distinguishes the simple 3-axis wrapper from the
    full-featured `PID`. PascalCase, consistent with project conventions.
  - `MYPID`: meaningless name ("my PID"), ALL_CAPS violates naming convention.
  - Decision: `BasicPID` in `basic_pid.hpp`. Satisfies M-001.

---

## Validation

[**Unit Tests**]

- V-UT-1: Deferred (NG-2).

[**Integration Tests**]

- V-IT-1: `colcon build` succeeds, zero warnings.
- V-IT-2: `colcon test` passes.

[**Failure / Robustness Validation**]

- V-F-1: Each `.hpp` compiles standalone.

[**Edge Case Validation**]

- V-E-1: `grep -r "using namespace Eigen" include/` → zero results.
- V-E-2: `grep -r "#undef MIN\|#undef MAX" include/ src/` → zero results.
- V-E-3: No duplicated `is_zero`/`constrain_float` across .cpp files.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1 |
| G-2 | V-IT-1 |
| G-3 | V-E-1 |
| G-4 | V-E-2 |
| G-5 | V-E-3 |
| G-6 | File/class naming review |
| C-1 | V-F-1 |
| C-2 | Debug macros in pid.hpp |
| C-3 | PID::Defaults struct unchanged |
| C-4 | math_types.hpp includes Eigen |
