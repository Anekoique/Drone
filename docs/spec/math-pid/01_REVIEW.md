# `Math & PID` REVIEW `01`

> Status: Open
> Feature: `math-pid`
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
- Blocking Issues: `2`
- Non-Blocking Issues: `2`

## Summary

Round 01 is closer to a workable phase boundary. Keeping the math split is the
right correction, and the plan is clearer than round 00 about what belongs in
`math_utils` versus the Eigen-heavy helper layer.

The blocking issues are still at the contract boundary. The plan claims it
resolved the roadmap alignment problem, but the current roadmap still targets
`mypid.hpp` while the plan now targets `basic_pid.hpp`. The plan also still
claims “all legacy function signatures preserved” even though the renamed class
and method surface no longer match the legacy code that later phases will
consume.

---

## Findings

### R-001 `Roadmap alignment is still not actually resolved`

- Severity: HIGH
- Section: `Review Adjustments / Master Compliance / Spec`
- Type: Spec Alignment
- Problem:
  The plan says R-001 was accepted and that the roadmap was updated to reflect
  the final file names, but the current roadmap still defines the Phase-2
  target for `MYPID.h` as `include/drone/math/mypid.hpp`. Round 01 instead
  targets `basic_pid.hpp` and `BasicPID`.
- Why it matters:
  The user asked to drive the phase from the roadmap. Until the roadmap and the
  phase plan name the same target, the plan is still diverging from the source
  of truth and the response matrix overstates what was actually resolved.
- Recommendation:
  Resolve this one way or the other in round 02:
  either update the roadmap first to adopt `basic_pid.hpp`, or keep the Phase-2
  target as `mypid.hpp` and defer the rename to a later cleanup round.

### R-002 `The plan still does not preserve the actual MYPID interface`

- Severity: HIGH
- Section: `Review Adjustments / Data Structure / API Surface / Implementation Plan`
- Type: Correctness
- Problem:
  The plan says “All legacy function signatures preserved,” but the documented
  API now renames both the class and the methods that matter to later
  consumers: legacy `MYPID::readPIDParameters()` becomes `BasicPID::load_params()`
  and legacy `MYPID::Mypid(...)` becomes `BasicPID::update(...)`.
- Why it matters:
  This is not a mechanical transfer anymore. The legacy state machine directly
  calls `mypid.readPIDParameters(...)`, `mypid.read_goal(...)`, mutates the
  public velocity fields, and invokes `mypid.Mypid(...)`. Without a
  compatibility story, later phases cannot migrate incrementally against the
  advertised Phase-2 output.
- Recommendation:
  For Phase 2, preserve the legacy public surface exactly, or add an explicit
  compatibility layer:
  keep `MYPID`/`readPIDParameters`/`Mypid` available and optionally provide
  `BasicPID` and `load_params`/`update` as aliases or wrappers.

### R-003 `Config-loading scope is internally contradictory`

- Severity: MEDIUM
- Section: `Summary / Review Adjustments / Non-goals / Step 4`
- Type: Maintainability
- Problem:
  The plan says there is “no config-path migration,” and the review-adjustment
  text says BasicPID and PID still use the legacy `Readyaml` path internally.
  But Step 4 then says BasicPID uses `drone::ConfigLoader::load()`, which is
  the migrated Phase-1 loader and not the legacy implementation.
- Why it matters:
  The executor cannot tell whether Phase 2 is supposed to preserve the old
  loading behavior or explicitly adopt the Phase-1 installed-layout loader.
  That ambiguity affects both implementation and validation.
- Recommendation:
  Pick one statement and make the plan consistent:
  either say Phase 2 adopts the Phase-1 `ConfigLoader` surface, or say config
  loading is untouched and keep the old method names/loader calls for now.

### R-004 `Validation still does not prove consumer compatibility`

- Severity: MEDIUM
- Section: `Validation / Acceptance Mapping`
- Type: Validation
- Problem:
  The plan still relies on `colcon build`, `colcon test`, grep, and file review.
  That can prove the package builds, but it does not prove the migrated Phase-2
  headers expose the contract later phases will actually consume.
- Why it matters:
  `colcon test` only runs registered tests. With no dedicated Phase-2 consumer
  smoke test, the package can pass validation while the exported `mypid`/`pid`
  interface is still incompatible with the legacy call sites the migration is
  supposed to preserve.
- Recommendation:
  Add a minimal compile-time or runtime smoke test for the exported surface,
  such as a tiny test/executable that includes the public headers and calls the
  preserved `PID` and `MYPID`-level APIs against one existing config file.

---

## Trade-off Advice

### TR-1 `Prefer compatibility aliases over immediate public rename`

- Related Plan Item: `T-1`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Prefer Option B
- Advice:
  If the team wants the cleaner `BasicPID` naming, introduce it as an alias or
  wrapper first instead of replacing the legacy public name in Phase 2.
- Rationale:
  This keeps the migration path incremental. Later phases can still consume the
  legacy surface mechanically, while the codebase can begin converging on the
  cleaner name without a hard break.
- Required Action:
  Either preserve the legacy public symbols in round 02, or justify why a hard
  rename is worth breaking the stated mechanical-transfer contract.

### TR-2 `Be explicit about loader behavior in this phase`

- Related Plan Item: `Step 4`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Need More Justification
- Advice:
  Do not describe the phase as “no config-path migration” if the plan actually
  standardizes on `drone::ConfigLoader`.
- Rationale:
  Adopting the Phase-1 loader is a reasonable choice, but it is still a
  behavior boundary that should be named explicitly rather than hidden under
  “no migration.”
- Required Action:
  Clarify the intended loader behavior in the next plan and align the
  non-goals, implementation steps, and validation around that single choice.

---

## Positive Notes

- Keeping `math_utils` and the Eigen-heavy helper layer separate is a better
  migration shape than the merged round-00 proposal.
- The plan now documents the `BasicPID` data surface more concretely than the
  previous round.
- The header hygiene goals remain appropriate for this phase.

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
- Reason: The plan still has an unresolved roadmap mismatch and does not yet
  define a mechanically compatible public API for the `MYPID` migration path.
