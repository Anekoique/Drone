# `Math & PID` PLAN `00`

> Status: Draft
> Feature: `math-pid`
> Iteration: `00`
> Owner: Executor
> Depends on:
> - Previous Plan: `none`
> - Review: `none`
> - Master Directive: `none`

---

## Summary

Transfer the math helper layer and PID controllers from the legacy codebase.
These modules depend on Phase 1 utilities (readyaml) and Eigen, and are
consumed by PosControl and mission logic in later phases.

## Log

[**Feature Introduce**]

Migrates 4 module groups (~2400 lines): math helpers (math_utils + math),
PID controller, MYPID (simple PID wrapper), and SCurve (waypoint path
planner). Mechanical transfer with targeted modernization: replace macros
with constexpr/std, remove `using namespace Eigen`, fix duplicated code.

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

- G-1: Transfer `math_utils`, `math`, `PID`, `MYPID`, and `SCurve` into
  `include/drone/math/` and `src/math/`.
- G-2: All code compiles as part of `drone_utils` library, C++20, zero
  warnings.
- G-3: Remove `using namespace Eigen` from all headers — use `Eigen::`
  prefix.
- G-4: Replace `MIN`/`MAX` macros and `#undef` hacks with
  `std::min`/`std::max`/`std::clamp`.
- G-5: Consolidate duplicated `is_zero`/`is_positive`/`is_negative`/
  `constrain_float` (defined in both `math.cpp` and `PID.cpp`) into a single
  source.
- G-6: `MYPID` uses `drone::ConfigLoader` instead of legacy `Readyaml`.

Non-goals:
- NG-1: No PID algorithm changes — transfer only.
- NG-2: No unit tests (deferred to testing feature).
- NG-3: No `#define` PID gain removal from PosControl.h — that belongs
  to Phase 5.

[**Architecture**]

```
include/drone/math/
├── math_utils.hpp      (from math_utils.h + math.h + math.cpp)
├── pid.hpp             (from PID.h)
├── simple_pid.hpp      (from MYPID.h, renamed)
└── scurve.hpp          (from SCurve.h)

src/math/
├── math_utils.cpp      (from math.cpp)
├── pid.cpp             (from PID.cpp)
└── scurve.cpp          (from SCurve.cpp)
```

`MYPID` renamed to `SimplePID` — header-only, uses `drone::ConfigLoader`.

[**Invariants**]

- I-1: No `using namespace Eigen` in any header.
- I-2: All headers use `#pragma once` and are self-contained.
- I-3: No duplicated function definitions across translation units.
- I-4: `is_zero`, `is_positive`, `is_negative`, `constrain_float`,
  `safe_sqrt`, `kinematic_limit` defined once in `math_utils`.
- I-5: No `#undef MIN`/`#undef MAX` macros.
- I-6: No behavioral changes to PID or SCurve algorithms.

[**Data Structure**]

| Legacy File | Target Header | Target Source | Changes |
|---|---|---|---|
| `math_utils.h` | `include/drone/math/math_utils.hpp` | `src/math/math_utils.cpp` | Merge with math.h/cpp |
| `math.h` | (merged into math_utils.hpp) | (merged into math_utils.cpp) | Remove `using namespace Eigen`, replace MIN/MAX |
| `math.cpp` | — | `src/math/math_utils.cpp` | Consolidate, remove duplication |
| `PID.h` | `include/drone/math/pid.hpp` | `src/math/pid.cpp` | `#pragma once`, namespace drone |
| `PID.cpp` | — | `src/math/pid.cpp` | Remove duplicated math fns, use math_utils |
| `MYPID.h` | `include/drone/math/simple_pid.hpp` | — (header-only) | Rename, use ConfigLoader |
| `SCurve.h` | `include/drone/math/scurve.hpp` | `src/math/scurve.cpp` | Remove `using namespace Eigen` |
| `SCurve.cpp` | — | `src/math/scurve.cpp` | Include path update |

[**API Surface**]

All function signatures preserved. Namespace `drone::` added. Key type
changes:
- `Vector3f`, `Vector4f`, `Quaternionf` used via `Eigen::` prefix in headers
- `MYPID` → `SimplePID`, method `readPIDParameters` → uses `ConfigLoader::load`
- `constrain_float` → `std::clamp` where possible

[**Constraints**]

- C-1: SCurve uses `Vector3f` extensively — Eigen types must remain accessible
  in the header without `using namespace`.
- C-2: PID `#ifdef` debug macros (`pid_debug_print`, `fuzzy_pid_dead_zone`)
  preserved — they are compile-time switches used during tuning.
- C-3: `PID::Defaults` struct and YAML loading via `readPIDParameters` must
  remain compatible with existing config files.

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create `include/drone/math/` and `src/math/` directories.
2. Transfer `math_utils` — merge `math_utils.h` + `math.h` + `math.cpp`.
3. Transfer `PID` — remove duplicated math fns from PID.cpp.
4. Transfer `MYPID` → `SimplePID` — use ConfigLoader.
5. Transfer `SCurve` — remove `using namespace Eigen`.
6. Update CMakeLists.txt — add new sources to `drone_utils`.
7. Build and verify.

[**Failure Flow**]

1. Compile errors → fix Eigen:: prefix, missing includes.

[**State Transition**]

Not applicable.

### Implementation Plan

[**Step 1 — math_utils**]

Merge `math_utils.h` (declarations) + `math.h` (templates, Eigen typedefs) +
`math.cpp` (implementations) into one cohesive module:

- `math_utils.hpp`: declarations + inline templates (`sq`, `norm`,
  `safe_sqrt`, `rotate_xy`, `is_equal` template). No `using namespace Eigen`.
  Replace `MIN`/`MAX` macros with `std::min`/`std::max`. Remove `#undef`.
  Remove the `isfinite` macro redefinition (C++20 has `std::isfinite`).
- `math_utils.cpp`: `is_zero`, `is_positive`, `is_negative`, `is_equal`
  (Vector4f overload), `constrain_float`, `kinematic_limit`, `safe_sqrt`
  explicit instantiations.

[**Step 2 — PID**]

- `pid.hpp`: `#pragma once`, `namespace drone`, Eigen prefix, keep
  `Defaults` struct and `PIDInfo` struct, keep `readPIDParameters` static
  (uses `ConfigLoader`).
- `pid.cpp`: Remove the duplicated `is_zero`/`constrain_float` block
  (`#ifndef MATH_H` guard). Include `drone/math/math_utils.hpp` instead.
  Remove commented-out `main()`. Remove trailing `#ifndef POSCONTROL_H`
  block with hardcoded defaults.

[**Step 3 — SimplePID (was MYPID)**]

- `simple_pid.hpp`: Header-only. Replace `Readyaml::readYAML` →
  `ConfigLoader::load`. Rename class `MYPID` → `SimplePID`.
  Use `std::clamp` instead of manual limit checks.

[**Step 4 — SCurve**]

- `scurve.hpp`: `#pragma once`, `namespace drone`. Replace `Vector3f` with
  `Eigen::Vector3f` in all signatures. Remove include of legacy `math.h` →
  include `drone/math/math_utils.hpp`.
- `scurve.cpp`: Include path updates. No logic changes.

[**Step 5 — CMakeLists.txt**]

Add to `drone_utils` sources:
```cmake
src/math/math_utils.cpp
src/math/pid.cpp
src/math/scurve.cpp
```

[**Step 6 — Build + verify**]

`colcon build` + `colcon test` via Docker.

## Trade-offs

- T-1: **Merge math_utils + math into one module vs keep separate**
  - Merged: single include for all math helpers, no split API surface.
  - Separate: matches legacy structure.
  - Decision: Merge. The split was artificial — `math_utils.h` had 3 declarations
    and `math.h` had the rest. They're one cohesive module.

- T-2: **Rename MYPID → SimplePID vs keep MYPID**
  - SimplePID: descriptive, follows naming conventions.
  - MYPID: preserves legacy familiarity.
  - Decision: Rename. The name `MYPID` is not descriptive and doesn't follow
    snake_case convention.

---

## Validation

[**Unit Tests**]

- V-UT-1: Deferred (NG-2).

[**Integration Tests**]

- V-IT-1: `colcon build --packages-select drone` succeeds, zero warnings.
- V-IT-2: `colcon test --packages-select drone` passes.

[**Failure / Robustness Validation**]

- V-F-1: Each new `.hpp` compiles standalone.

[**Edge Case Validation**]

- V-E-1: `grep -r "using namespace Eigen" include/` returns zero results.
- V-E-2: `grep -r "#undef MIN\|#undef MAX" include/ src/` returns zero results.
- V-E-3: No duplicated `is_zero`/`constrain_float` definitions across .cpp files.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1 |
| G-2 | V-IT-1 (zero warnings) |
| G-3 | V-E-1 |
| G-4 | V-E-2 |
| G-5 | V-E-3 |
| G-6 | SimplePID uses ConfigLoader (code review) |
| C-1 | V-F-1, V-E-1 |
| C-2 | Debug macros preserved (code review) |
| C-3 | PID::Defaults struct unchanged |
