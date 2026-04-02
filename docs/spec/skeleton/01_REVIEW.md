# `Skeleton` REVIEW `01`

> Status: Open
> Feature: `skeleton`
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

Round 01 fixes the major scope problems from round 00. The package is now a
true phase-0 skeleton instead of a speculative migration shell, and removing
launch/config renaming from this phase is a cleaner cut. One blocking issue
remains in the package manifest specification, and there are two smaller gaps in
validation and document consistency.

---

## Findings

### R-001 `package.xml spec omits the ament_cmake build-type export`

- Severity: HIGH
- Section: `Implement / Step 1`
- Type: Correctness
- Problem:
  The plan describes `package.xml` as format 3 with `ament_cmake` buildtool
  dependency and test dependencies, but it never specifies the required
  `<export><build_type>ament_cmake</build_type></export>` stanza.
- Why it matters:
  The build type is how ROS 2 tooling identifies the package as an
  `ament_cmake` package. Omitting it leaves the manifest incomplete for the
  package type the plan is explicitly trying to create.
- Recommendation:
  Update the `package.xml` specification and validation checklist to include the
  `ament_cmake` build-type export explicitly.

### R-002 `Config installation is designed but not actually validated`

- Severity: MEDIUM
- Section: `Implementation Plan / Validation`
- Type: Validation
- Problem:
  The plan includes `install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)`
  and treats copied configs as a core phase-0 deliverable, but validation only
  checks source-file byte identity and build/test success. It never verifies the
  installed package layout.
- Why it matters:
  A passing build does not prove the config files are packaged correctly. Since
  the configs are one of the few real artifacts in this phase, install-layout
  validation should be explicit.
- Recommendation:
  Add a validation step that checks `install/<prefix>/share/drone/config/`
  contains the five YAML files and that they match the copied source files.

### R-003 `Review-adjustment language is inconsistent with the revised design`

- Severity: LOW
- Section: `Log / Review Adjustments / Changes from Previous Round`
- Type: Maintainability
- Problem:
  The plan says “CMakeLists.txt has no `find_package` calls” and “package.xml
  lists only `ament_cmake`”, but the implementation section later includes
  `find_package(ament_cmake REQUIRED)` and test dependencies.
- Why it matters:
  This is not a design flaw, but it makes the round harder to audit because the
  summary language overstates how far the dependency surface was reduced.
- Recommendation:
  Rephrase those lines to say there are no runtime or future-phase dependencies,
  rather than no dependencies at all.

---

## Trade-off Advice

_(none)_

---

## Positive Notes

- The round-01 plan resolves the two blocking issues from round 00 cleanly.
- Keeping legacy config filenames in phase 0 is the right scope decision.
- The reduced package definition is much easier to validate and migrate from in
  later phases.

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
- Reason: The manifest spec is still incomplete for an `ament_cmake` package.
