# `Skeleton` REVIEW `00`

> Status: Open
> Feature: `skeleton`
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
- Non-Blocking Issues: `1`

## Summary

The phase-0 plan is organized well and the scope is mostly coherent, but two
blocking issues remain in the dependency model. The plan currently ties
dependency resolution to the wrong source of truth and does not account for the
runtime dependencies introduced by the new Python launch file. Until those are
fixed, phase 0 is not reliably buildable or launchable as specified.

---

## Findings

### R-001 `Dependency resolution is attached to the wrong source of truth`

- Severity: HIGH
- Section: `Constraints / Implementation Plan / Validation`
- Type: Correctness
- Problem:
  C-1 says legacy `find_package(...)` entries must be added to `CMakeLists.txt`
  "so that `rosdep` can resolve them". That is not how ROS dependency
  resolution works. `rosdep` resolves keys from `package.xml`, not from CMake.
  For an empty skeleton package, forcing all future legacy dependencies into
  CMake also makes phase 0 fail for reasons unrelated to the phase-0 artifact
  set.
- Why it matters:
  This weakens the phase boundary. A phase intended to establish a minimal
  package skeleton becomes coupled to the full future migration dependency
  closure.
- Recommendation:
  Make `package.xml` the dependency source of truth for `rosdep`. Keep CMake
  limited to dependencies actually needed by the phase-0 package contents, or
  explicitly separate current-phase dependencies from future migration
  dependencies.

### R-002 `Launch-file dependencies introduced by the plan are not declared`

- Severity: HIGH
- Section: `API Surface / Implementation Plan / Validation`
- Type: Spec Alignment
- Problem:
  The plan introduces a Python launch file that uses launch arguments and
  `ament_index_python`, but the dependency strategy only talks about mirroring
  legacy build dependencies. The plan never adds the runtime dependencies
  required by this new launch infrastructure.
- Why it matters:
  `ros2 launch drone drone.launch.py` is an acceptance criterion. Without
  declaring the launch-related runtime dependencies in `package.xml`, phase 0
  can satisfy the file layout but still fail the stated launch goal.
- Recommendation:
  Add the launch-runtime dependencies explicitly to the plan. At minimum,
  account for `launch`, `ament_index_python`, and `ros2launch`; add
  `launch_ros` only if phase 0 actually creates ROS node actions in the launch
  file.

### R-003 `Validation does not prove dependency parity`

- Severity: MEDIUM
- Section: `Validation / Acceptance Mapping`
- Type: Validation
- Problem:
  C-2 requires `package.xml` and `CMakeLists.txt` to match exactly, but the
  plan maps that to `ament_lint` success. That test does not prove exact parity
  between manifest dependencies and CMake dependency declarations.
- Why it matters:
  A core constraint is currently unverifiable under the proposed validation
  plan. That leaves a gap exactly where the phase is trying to establish
  build-system discipline.
- Recommendation:
  Add an explicit validation step for dependency parity, such as a manual
  checklist or a small scripted comparison between declared package
  dependencies and active CMake dependencies.

---

## Trade-off Advice

### TR-1 `Config rename timing`

- Related Plan Item: `T-2`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Need More Justification
- Advice:
  Do not rename config files in phase 0 unless the rename is required by some
  concrete package or launch constraint.
- Rationale:
  This phase explicitly has no migrated runtime code. Renaming files now
  increases migration churn and breaks one-to-one traceability without
  delivering behavior.
- Required Action:
  Either keep legacy config filenames in `00_PLAN.md`, or justify why the
  rename must happen before any functional code migration.

### TR-2 `C++17 baseline`

- Related Plan Item: `T-1`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Prefer Option A
- Advice:
  Keep the move to C++17.
- Rationale:
  The legacy code already uses `std::optional`, so the newer standard matches
  the actual codebase direction.
- Required Action:
  Keep as-is, but cite the existing C++17 usage as the justification in the
  next plan revision.

---

## Positive Notes

- The plan has a clear phase boundary: goals, non-goals, and invariants are
  explicit.
- The acceptance mapping is present and materially improves reviewability.
- The trade-off section is concrete and useful rather than decorative.

---

## Approval Conditions

### Must Fix
- R-001
- R-002

### Should Improve
- R-003

### Trade-off Responses Required
- TR-1

### Ready for Implementation
- No
- Reason: The dependency model and launch-runtime dependency set are not yet
  complete.
