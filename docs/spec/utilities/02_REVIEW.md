# `Utilities` REVIEW `02`

> Status: Open
> Feature: `utilities`
> Iteration: `02`
> Owner: Reviewer
> Target Plan: `02_PLAN.md`
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

Round 02 fixes the prior `Readyaml` path problem and moves the utility layer to
the right long-term header layout under `include/drone/`. The remaining blocker
is in the exported-library contract: the plan now promises cross-package use,
but it still does not export the `yaml-cpp` dependency required by the public
`readyaml.hpp` header.

---

## Findings

### R-001 `The exported utility package still does not export yaml-cpp to downstream consumers`

- Severity: HIGH
- Section: `Summary / Step 4 / Step 5 / Validation`
- Type: Correctness
- Problem:
  `readyaml.hpp` is a public installed header and directly includes
  `yaml-cpp/yaml.h`, returns `YAML::Node`, and performs inline YAML loading.
  However, the CMake plan only exports `Eigen3` and `ament_index_cpp` via
  `ament_export_dependencies(...)`. It does not `find_package(yaml-cpp
  REQUIRED)` or export `yaml-cpp` for downstream packages.
- Why it matters:
  The main reason given for the new `include/drone/` model is cross-package
  reuse. Under the current plan, a downstream package including
  `drone/utils/readyaml.hpp` would still need to rediscover and relink
  `yaml-cpp` manually, which breaks the exported-library contract.
- Recommendation:
  Add `find_package(yaml-cpp REQUIRED)` to the CMake plan and export it as a
  downstream dependency alongside `ament_index_cpp` and `Eigen3`. Keep the
  `package.xml` dependency entry as well.

### R-002 `Validation still does not prove downstream package consumption works`

- Severity: MEDIUM
- Section: `Validation`
- Type: Validation
- Problem:
  V-IT-4 only checks that headers exist under `install/.../include/drone/`.
  That proves installation, but not that another package can actually
  `find_package(drone)` and compile against the exported headers and library.
- Why it matters:
  This round’s core design change is exported cross-package reuse. Without a
  downstream smoke test, the review cannot verify the exact behavior that this
  round is introducing.
- Recommendation:
  Add a minimal downstream consumer validation step: a tiny test package or a
  CMake smoke target that uses `find_package(drone REQUIRED)` and includes
  `drone/utils/readyaml.hpp`.

### R-003 `The dependency header references a master artifact that does not exist`

- Severity: LOW
- Section: `Header / Depends on`
- Type: Maintainability
- Problem:
  `02_PLAN.md` lists `01_MASTER.md` as its master dependency, but there is no
  [01_MASTER.md](/Users/anekoique/tmp/drone/Drone/docs/spec/utilities/01_MASTER.md)
  artifact in this feature directory.
- Why it matters:
  This is a workflow/documentation error rather than a design flaw, but it makes
  the iteration chain harder to audit and conflicts with the template’s
  `none`-when-absent convention.
- Recommendation:
  Change the `Master Directive` line in `02_PLAN.md` to `none` unless a real
  `01_MASTER.md` artifact is created.

---

## Trade-off Advice

_(none)_

---

## Positive Notes

- The round-02 plan resolves the `Readyaml` installed-layout issue correctly.
- Moving public headers to `include/drone/` matches the updated project
- architecture and the modern target layout in [ROADMAP.md](/Users/anekoique/tmp/drone/Drone/docs/ROADMAP.md#L11).
- The package manifest is materially better than in round 01.

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
- Reason: The exported-library model is still incomplete for downstream
  consumers because `yaml-cpp` is not exported with the public `readyaml.hpp`
  header surface.
