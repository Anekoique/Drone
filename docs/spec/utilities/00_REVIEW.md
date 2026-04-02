# `Utilities` REVIEW `00`

> Status: Open
> Feature: `utilities`
> Iteration: `00`
> Owner: Reviewer
> Target Plan: `00_PLAN.md`
> Review Scope:
>
> - Plan Correctness
> - Spec Alignment
> - Design Soundness
> - Validation Adequacy
> - Trade-off Advice

---

## Verdict

- Decision: Approved with Revisions
- Blocking Issues: `2`
- Non-Blocking Issues: `2`

## Summary

The phase boundary is directionally good: this is the right place to start
migrating non-node C++ code. The main problems are that the proposed math split
does not match the real utility boundary in the legacy/reference code, and the
library packaging model is internally inconsistent. Those two issues should be
fixed before implementation starts.

---

## Findings

### R-001 `The proposed math split does not match the actual utility boundary`

- Severity: HIGH
- Section: `Goals / Architecture / Data Structure / Step 2`
- Type: Spec Alignment
- Problem:
  The plan treats `math_utils` as a tiny three-function module by pairing
  `math_utils.h` with only `is_zero`, `is_positive`, and `is_negative`
  extracted from `math.cpp`. But the real generic math helper layer is split
  across `math_utils.h` and `math.h` / `math.cpp`, and it includes
  `is_equal`, `constrain_float`, `kinematic_limit`, and `safe_sqrt`. The
  `DroneCompetition` reference preserves that two-file split as
  `math_utils_t.h` plus `math_t.h`.
- Why it matters:
  This phase would create an artificial API that does not reflect either the
  legacy utility boundary or the reference project. When `SCurve`, `PID`, and
  `PosControl` migrate later, the math layer will need to be redesigned again,
  causing churn and likely duplicated helpers.
- Recommendation:
  Either migrate the full generic math helper layer together
  (`math_utils.h` + `math.h` + `math.cpp`) or defer all math-helper migration
  to a later phase. Do not introduce a truncated `math_utils.cpp` containing
  only three predicates.

### R-002 `The planned library packaging contract is internally inconsistent`

- Severity: HIGH
- Section: `Architecture / Step 5 / Step 6 / Validation`
- Type: Correctness
- Problem:
  The plan places headers under `src/`, but the CMake snippet uses
  `$<INSTALL_INTERFACE:include>` as if the library were installed with public
  headers under `include/`. No header install/export rules are specified.
  At the same time, the plan uses
  `ament_target_dependencies(drone_utils Eigen3 yaml-cpp ament_index_cpp)`,
  while the legacy and `DroneCompetition` builds handle `yaml-cpp` via
  `target_link_libraries(...)` rather than as a normal ament dependency.
- Why it matters:
  The executor cannot implement this cleanly without making undocumented design
  decisions. The plan currently mixes two different models:
  an internal static helper library and an installed reusable package library.
  That ambiguity is likely to produce incorrect CMake/package behavior in the
  first code-bearing phase.
- Recommendation:
  Choose one model explicitly.
  If the library is internal-only for now, keep headers in `src/`, drop the
  install-interface/export assumptions, and document the exact link pattern for
  non-ament libraries.
  If the library is meant to be reusable, move public headers to
  `include/drone/...`, add install/export rules, and validate the installed
  package layout.

### R-003 `Dead-code deletion is outside this feature's actual scope`

- Severity: MEDIUM
- Section: `Goals / Data Structure`
- Type: Maintainability
- Problem:
  G-4 says this phase removes `AC_WPNav.h/cpp`, `Yolo.h.bak`, and
  `Yolo copy.bak`. Those files are in the legacy tree, not in the new `Drone`
  package tree described by this feature.
- Why it matters:
  This feature is supposed to define the contents of the new package. Mixing in
  destructive cleanup of the legacy source tree makes the phase boundary less
  clear and complicates review/history.
- Recommendation:
  Rephrase this as “do not migrate these files” or move legacy cleanup into a
  dedicated hygiene feature.

### R-004 `Build-only validation is too weak for the first algorithm migration`

- Severity: MEDIUM
- Section: `Non-goals / Validation`
- Type: Validation
- Problem:
  The plan introduces the first substantial C++ algorithmic code into `Drone`
  and also modernizes comments, header boundaries, and code style, but it
  explicitly defers all unit tests. The current validation is almost entirely
  build-based.
- Why it matters:
  `FuzzyPID`, `AutoTune`, `TrajectoryGenerator`, and `Readyaml` are
  deterministic enough to support lightweight characterization tests. Without
  them, this phase cannot distinguish “compiles” from “behavior preserved”.
- Recommendation:
  Add at least a minimal characterization test set for a few migrated utility
  modules, or narrow the phase further to mechanical relocation only and defer
  semantic cleanup until tests exist.

---

## Trade-off Advice

### TR-1 `Internal library vs reusable exported library`

- Related Plan Item: `T-1`
- Topic: Simplicity vs Reuse
- Reviewer Position: Prefer internal-only first
- Advice:
  Treat `drone_utils` as an internal package library in this phase.
- Rationale:
  There is no downstream package consumer yet. Keeping the library internal
  avoids premature include/export/install design while the module boundaries are
  still being stabilized.
- Required Action:
  Update the plan to describe an internal library model explicitly, or justify
  why exported reuse is needed in this phase.

### TR-2 `Math migration scope`

- Related Plan Item: `G-1`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Need More Justification
- Advice:
  Keep the math helper layer cohesive instead of partially extracting it.
- Rationale:
  The reference project and legacy tree both preserve a broader math utility
  surface than the proposed three-function subset.
- Required Action:
  Either migrate the whole math helper surface or explain why the truncated
  split will not create a second refactor when control modules migrate.

---

## Positive Notes

- The phase goal of starting with non-node C++ modules is sound.
- The domain-grouped directory layout is a better long-term target than a flat
  `src/` dump.
- Preserving the current `FuzzyPID` macro surface for now is a pragmatic call.

---

## Approval Conditions

### Must Fix
- R-001
- R-002

### Should Improve
- R-003
- R-004

### Trade-off Responses Required
- TR-1
- TR-2

### Ready for Implementation
- No
- Reason: The math-module boundary and the library packaging model are not yet
  well specified enough for implementation.
