# `Utilities` REVIEW `01`

> Status: Open
> Feature: `utilities`
> Iteration: `01`
> Owner: Reviewer
> Target Plan: `01_PLAN.md`
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
- Blocking Issues: `1`
- Non-Blocking Issues: `2`

## Summary

Round 01 resolves the main structural problems from round 00: the partial math
split is gone, the library is clearly internal-only, and the scope now matches
the roadmap much better. One blocking issue remains around `Readyaml`, and the
dependency/validation model still has two smaller gaps.

---

## Findings

### R-001 `Readyaml is still specified with the legacy broken path model`

- Severity: HIGH
- Section: `Summary / Goals / Data Structure / Step 2`
- Type: Correctness
- Problem:
  The plan reduces `Readyaml` changes to a package-name substitution
  (`"px4_ros_com"` → `"drone"`), but does not require fixing the legacy config
  lookup behavior. The legacy `Readyaml.h` resolves
  `.../../../../../src/<pkg>/config/<file>`, while the current `Drone`
  skeleton installs config files to `share/drone/config/`. The reference
  `DroneCompetition` version already switched to the installed-layout lookup.
- Why it matters:
  This utility would be migrated with known-broken runtime behavior for the new
  package layout. Later phases would then depend on a config loader that only
  works in a source-tree-specific workspace arrangement.
- Recommendation:
  Make installed-layout config lookup part of this phase for `readyaml.hpp`:
  resolve files from `ament_index_cpp::get_package_share_directory("drone") +
  "/config/" + filename`, and validate against the installed package layout.

### R-002 `The package dependency model is still incomplete for yaml-cpp`

- Severity: MEDIUM
- Section: `Step 3 / Step 4`
- Type: Correctness
- Problem:
  `readyaml.hpp` directly includes `yaml-cpp/yaml.h`, and the library target
  links `yaml-cpp`, but the plan explicitly says no `package.xml` dependency is
  needed because the library is linked directly. That is not consistent with the
  ROS dependency model, where `package.xml` is expected to contain the complete
  dependency set used by the package.
- Why it matters:
  Fresh environments relying on `rosdep` and package metadata can miss a direct
  system dependency even though the code and CMake use it. This undermines the
  phase goal of keeping each step buildable in isolation.
- Recommendation:
  Add the appropriate `yaml-cpp` rosdep key to `package.xml`, and keep the
  direct CMake link step if desired.

### R-003 `Eigen dependency handling is only half-specified`

- Severity: LOW
- Section: `Step 3 / Step 4`
- Type: Maintainability
- Problem:
  The plan adds `<buildtool_depend>eigen3_cmake_module</buildtool_depend>` in
  `package.xml`, but the CMake snippet only shows `find_package(Eigen3
  REQUIRED)`. That leaves the declared build-tool dependency and the CMake model
  slightly out of sync.
- Why it matters:
  This probably won’t block implementation on a typical Ubuntu 22.04 machine,
  but it leaves the plan less precise than it should be for the first code
  migration round.
- Recommendation:
  Either add `find_package(eigen3_cmake_module REQUIRED)` to the CMake plan or
  justify why the extra manifest dependency is intentionally not consumed in
  CMake.

---

## Trade-off Advice

_(none)_

---

## Positive Notes

- The round-01 plan correctly removes the partial `math_utils` transfer.
- The internal-only library decision is much cleaner than the round-00 mixed
  export/install model.
- The scope now matches the roadmap and master directives substantially better.

---

## Approval Conditions

### Must Fix
- R-001

### Should Improve
- R-002
- R-003

### Trade-off Responses Required
_(none)_

### Ready for Implementation
- No
- Reason: `Readyaml` still needs to be specified for the installed `Drone`
  package layout rather than the legacy source-tree layout.
