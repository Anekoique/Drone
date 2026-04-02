# `Math & PID` REVIEW `00`

> Status: Open
> Feature: `math-pid`
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

The phase boundary is mostly sensible: moving the math helpers and PID layer
before `PosControl` and mission code matches the bottom-up roadmap. The plan
also identifies real cleanup work in the legacy math layer, especially the
duplicated helper functions and the `using namespace Eigen` leakage.

The blocking problems are at the interface boundary. Round 00 is framed as a
mechanical transfer phase, but the plan currently changes the roadmap contract
by renaming `MYPID`, collapsing the math module boundary, and leaving the
compatibility story incomplete for later consumers. That makes Phase 2 harder
to execute as an isolated migration step.

---

## Findings

### R-001 `Plan diverges from the phase-2 roadmap contract`

- Severity: HIGH
- Section: `Spec / Architecture / Data Structure / Trade-offs`
- Type: Spec Alignment
- Problem:
  The roadmap defines Phase 2 targets as `math_utils`, `pid`, `mypid`, and
  `scurve`, with `mypid` explicitly landing at `include/drone/math/mypid.hpp`.
  The plan changes that contract by renaming `MYPID` to `SimplePID` and by
  collapsing `math.h` into a single `math_utils` module.
- Why it matters:
  The user explicitly asked to drive the work from the roadmap. If the first
  plan for Phase 2 changes module names and boundaries without first updating
  the roadmap, later phases lose a stable reference point and the migration
  ceases to be mechanical.
- Recommendation:
  Keep the roadmap module/file contract in round 00:
  `mypid.hpp` should remain the Phase 2 target, and the math layer should
  either preserve a visible `math` boundary or the roadmap should be revised in
  a separate round before the plan adopts the merged layout.

### R-002 `The compatibility surface for MYPID is underspecified`

- Severity: HIGH
- Section: `API Surface / Constraints / Implementation Plan`
- Type: Correctness
- Problem:
  The plan says “All function signatures preserved” and “No PID algorithm
  changes,” but it also renames `MYPID`, switches it to `ConfigLoader`, and
  reduces the migration description to “simple PID wrapper.” The legacy class
  exposes more than a constructor and a compute function: it has
  `readPIDParameters`, `read_goal`, public state fields, and a `Mypid(...)`
  convenience method that are consumed directly by the state machine.
- Why it matters:
  Later phases depend on this surface for a mechanical migration path. Today
  `FlyState::MYPID` and `owner_->mypid` call `read_goal`, mutate the public
  velocity fields, and invoke `Mypid(...)` directly. A rename-only plan does
  not describe whether those members are preserved, wrapped, or intentionally
  broken.
- Recommendation:
  Expand the API section to list the actual preserved surface of `MYPID`, or
  keep the class name and file name unchanged in Phase 2. If a cleaner `SimplePID`
  name is desired, add it later behind a compatibility alias or in a dedicated
  cleanup iteration.

### R-003 `Validation does not prove config-path or runtime compatibility`

- Severity: MEDIUM
- Section: `Validation / Acceptance Mapping`
- Type: Validation
- Problem:
  The plan defers unit tests and relies on `colcon build`, `colcon test`, grep,
  and “code review” to validate `ConfigLoader` adoption and YAML compatibility.
  That is too weak for the changes proposed here, because the package currently
  has no Phase-2 runtime tests that exercise `PID::readPIDParameters`,
  `MYPID::read_goal`, or SCurve helper behavior against installed configs.
- Why it matters:
  `colcon test` only runs tests that are actually registered. In this package,
  passing `colcon test` would not demonstrate that the migrated config-loading
  path and math/PID interfaces still behave as expected.
- Recommendation:
  Add at least a minimal smoke validation step for Phase 2:
  a small compile-and-run test or executable that loads one existing config file
  through the migrated API and instantiates `PID` and `MYPID`/`mypid`.
  If that is intentionally out of scope, move the loader migration out of this
  phase and keep the transfer purely structural.

### R-004 `The math_utils template plan is internally inconsistent`

- Severity: MEDIUM
- Section: `Implementation Plan`
- Type: Maintainability
- Problem:
  Step 1 says `safe_sqrt` will become an inline template in
  `math_utils.hpp`, while `math_utils.cpp` will also keep explicit
  `safe_sqrt` instantiations. Those are two different implementation models,
  and the plan does not choose one.
- Why it matters:
  This creates avoidable ambiguity before implementation starts. The executor
  should not have to guess whether `safe_sqrt` is header-only or split across
  header/source with explicit instantiations.
- Recommendation:
  Pick one approach and state it explicitly in the plan:
  either make `safe_sqrt` fully header-only, or keep the declaration in the
  header and the definition plus explicit instantiations in the `.cpp`.

---

## Trade-off Advice

### TR-1 `Prefer compatibility-first naming in Phase 2`

- Related Plan Item: `T-2`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Prefer Option B
- Advice:
  Keep `MYPID` as the Phase-2 migration target.
- Rationale:
  This phase is still building the lower layers that later mission/control code
  will consume. Preserving the legacy name and file target keeps the migration
  graph simple. A rename to `SimplePID` is cosmetic right now and expands the
  surface area of the phase for little technical benefit.
- Required Action:
  Adopt the compatibility-first choice in the next plan, or justify why the
  rename must happen before any higher-level migration begins.

### TR-2 `Keep the math split unless the merge is justified by consumers`

- Related Plan Item: `T-1`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Need More Justification
- Advice:
  Do not merge `math_utils` and `math` in round 00 unless the plan shows which
  public includes later phases will consume and why the split is harmful.
- Rationale:
  The legacy split is awkward, but it is also the shape that `PID` and `SCurve`
  were written against. For a transfer phase, a thin compatibility layer is
  usually safer than a merged public surface.
- Required Action:
  Either preserve the split now, or add a clearer justification and
  compatibility plan for the merged layout.

---

## Positive Notes

- The bottom-up sequencing is correct: moving math/PID before `PosControl` and
  mission logic matches the roadmap dependency direction.
- Consolidating duplicated helper functions out of `PID.cpp` is worthwhile and
  should reduce future drift.
- Removing `using namespace Eigen` from public headers is the right invariant to
  establish early.

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
- Reason: The plan still changes the roadmap/API contract before the phase-2
  migration boundary is stable.
