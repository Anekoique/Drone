# `Perception` REVIEW `01`

> Status: Open
> Feature: `perception`
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
- Blocking Issues: `3`
- Non-Blocking Issues: `1`

## Summary

Round 01 fixes the main round-00 detector gaps well. The TensorRT plugin is now
modeled explicitly, the CUDA discovery strategy is modernized, and the cv_py
behavior split is much clearer. The plan is substantially closer to an
implementable Jetson perception phase.

The remaining blockers are at the package boundary. The public perception API
now exposes OpenCV and ROS types, but the dependency/export contract is still
missing from the package plan. The clustering rewrite also drops the legacy
diameter-aware data model even though that diameter is part of the current shot
selection behavior. Finally, the target layout is still internally inconsistent:
non-CUDA perception is merged into `drone_utils`, while CUDA perception becomes
`drone_perception_cuda`, which breaks the roadmap's intended module boundary.

---

## Findings

### R-001 `Public perception headers add undeclared and unexported dependencies`

- Severity: HIGH
- Section: `API Surface / Step 1 / Step 6 / Step 12 / Validation`
- Type: API
- Problem:
  The plan's public API now exposes `cv::Mat`, `vision_msgs::msg::Detection2DArray`,
  and `rclcpp::Time`, but the implementation plan only adds `find_package(OpenCV REQUIRED)`
  and then appends the non-CUDA sources to `drone_utils`. It does not specify the
  required `package.xml` dependencies or the `ament` export changes for OpenCV,
  `vision_msgs`, and `rclcpp`. The current package metadata only exports
  `Eigen3`, `ament_index_cpp`, and `yaml-cpp`.
- Why it matters:
  This makes the advertised installed headers unusable for downstream consumers.
  ROS dependency resolution is driven from `package.xml`, and exported libraries
  need their public build dependencies declared and exported consistently.
- Recommendation:
  In round 02, make the dependency contract explicit:
  add the required package manifest entries, add the needed `find_package(...)`
  and `ament_target_dependencies(...)` calls, and export any public dependencies
  from the target that installs these headers. If the project should stay ROS-free
  at the installed header boundary, move `Detection2DArray` conversion behind a
  private adapter instead of a public header.

### R-002 `The clustering plan no longer preserves the legacy diameter-aware behavior`

- Severity: HIGH
- Section: `API Surface / Step 5 / Validation`
- Type: Correctness
- Problem:
  The plan changes clustering input to `std::vector<Eigen::Vector3d>` and
  `find_cluster_centers(const std::vector<Eigen::Vector3d>& points)`, but the
  legacy algorithm does not cluster on position alone. `clustering.cpp` uses
  `Circles::diameters` as part of normalization, distance computation, and center
  updates, and the mission logic later sorts cluster centers by diameter before
  choosing the shot order.
- Why it matters:
  This is not just a dependency cleanup. It changes the clustering semantics and
  removes information that the current mission logic uses directly. The new API
  still returns a `Target` with `diameter`, but the plan no longer defines where
  that diameter comes from.
- Recommendation:
  Keep the boundary break, but preserve the data model. Round 02 should introduce
  a small perception-local sample type that carries both position and diameter,
  and the validation plan should include a fixed-input clustering case that checks
  both center position and recovered diameter/ranking behavior.

### R-003 `The target model still collapses perception into the utility library`

- Severity: HIGH
- Section: `Architecture / Step 6 / Step 12`
- Type: Spec Alignment
- Problem:
  The roadmap defines `perception` as its own architectural layer and module, but
  round 01 still puts the entire non-CUDA perception stack into `drone_utils`
  and only creates a separate target for the CUDA detector. That means one module
  is split across two unrelated targets: `drone_utils` for camera/filter/model/
  clustering/adapter, and `drone_perception_cuda` for TensorRT.
- Why it matters:
  This breaks the dependency direction the roadmap is trying to establish and
  drags OpenCV/ROS-facing perception concerns down into the utility target that
  earlier phases created for generic math/control helpers. It also leaves the
  downstream link contract unclear for later phases.
- Recommendation:
  Round 02 should introduce a dedicated non-CUDA perception target such as
  `drone_perception`, and then optionally layer `drone_perception_cuda` on top of
  it for Jetson-only detector code. If the project intentionally wants a single
  umbrella library target, the plan needs to justify that architectural change
  explicitly against the roadmap.

### R-004 `Validation still does not prove the installed consumer contract`

- Severity: MEDIUM
- Section: `Validation / Acceptance Mapping`
- Type: Validation
- Problem:
  `V-IT-3` says “All perception headers compile standalone”, but that only checks
  source-tree compilation. It does not verify that an external consumer can
  `find_package(drone)`, include the installed perception headers, and link the
  exported target set successfully with or without CUDA present.
- Why it matters:
  This phase is defining the public perception boundary for later modules. A
  compile-only check inside the same package will not catch missing exports,
  missing manifest dependencies, or an incoherent split between the base and CUDA
  perception targets.
- Recommendation:
  Add one downstream consumer smoke test in round 02:
  either a tiny test package or a CMake `try_compile`/sample executable that uses
  the installed perception headers and links the exported target(s) in both the
  non-CUDA and CUDA-enabled configurations.

---

## Trade-off Advice

### TR-1 `Prefer a dedicated perception base target`

- Related Plan Item: `Step 6 / Step 12`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Prefer Option A
- Advice:
  Split the non-CUDA perception code into its own base target instead of extending
  `drone_utils`.
- Rationale:
  The roadmap already establishes `perception` as a distinct layer. Preserving
  that boundary now will keep later driver/mission work cleaner and avoid turning
  `drone_utils` into a catch-all library.
- Required Action:
  Adopt a `drone_perception` base target in the next plan, or justify why a
  single umbrella target is a better long-term package contract.

### TR-2 `Prefer a richer clustering sample type over raw points`

- Related Plan Item: `Step 5`
- Topic: Fidelity vs Simplicity
- Reviewer Position: Prefer Option A
- Advice:
  Use a small struct such as `TargetSample { position, diameter }` rather than
  reducing clustering input to raw `Eigen::Vector3d` points.
- Rationale:
  The legacy behavior is explicitly diameter-aware, and the mission layer uses the
  resulting diameter ordering. A slightly richer type keeps the algorithm honest
  without reintroducing any `OffboardControl` coupling.
- Required Action:
  Adopt the richer sample type in round 02, or explain exactly how diameter-based
  target ordering will be preserved after the simplified API change.

---

## Positive Notes

- The TensorRT plugin problem from round 00 is now modeled concretely and in the
  right place.
- The `cv_py` kept/deferred/dropped table is much clearer and materially improves
  the phase boundary.
- The shift from deprecated `FindCUDA` to first-class CUDA language support plus
  `CUDAToolkit` is the right direction for the Jetson build split.

---

## Approval Conditions

### Must Fix
- R-001
- R-002
- R-003

### Should Improve
- R-004

### Trade-off Responses Required
- TR-1
- TR-2

### Ready for Implementation
- No
- Reason: The public package boundary and clustering semantics are still not
  stable enough to implement Phase 3 safely.

Checked against official documentation:
- [ROS 2 rosdep tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Rosdep.html)
- [ament_cmake_target_dependencies](https://docs.ros.org/en/humble/p/ament_cmake_target_dependencies/)
