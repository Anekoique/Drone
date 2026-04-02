# `Perception` REVIEW `00`

> Status: Open
> Feature: `perception`
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
- Blocking Issues: `3`
- Non-Blocking Issues: `2`

## Summary

The phase split is directionally correct. Pulling camera capture, detector,
tracking, geometry, and clustering together before the MAVROS and mission
layers matches the roadmap and removes several bad legacy boundaries, especially
the `OffboardControl` and ROS entanglement inside perception.

The blocking issues are around the detector contract. The current plan ports
only part of the `cv_cpp/trtdet` runtime, but the existing engines are coupled
to a custom TensorRT plugin library. The plan also promises a
`vision_msgs::msg::Detection2DArray` output without defining any adapter or
publisher surface that actually produces it. Finally, the proposed CUDA/CMake
strategy relies on deprecated `FindCUDA` behavior instead of current first-class
CUDA tooling, which weakens the core “Jetson yes, Docker/CI no” build goal.

---

## Findings

### R-001 `TensorRT plugin dependency is missing from the migration plan`

- Severity: HIGH
- Section: `Architecture / Data Structure / Step 3 / Step 7`
- Type: Correctness
- Problem:
  The plan ports `trtInfer.cpp`, `preprocess.cu`, and `postprocess.cpp`, but
  omits the `plugin/yololayer.*` runtime dependency and does not specify a
  plugin registration strategy. The current `cv_cpp/trtdet` package builds a
  separate `myplugins` shared library and links it into the inference
  executable, which strongly suggests the serialized engines depend on custom
  plugin creators during deserialization.
- Why it matters:
  TensorRT engines that depend on custom plugins are not self-contained at
  runtime. If the plugin library is not built and its creators are not
  registered before engine deserialization, the migrated detector can fail even
  if the rest of the code compiles cleanly.
- Recommendation:
  Add an explicit plugin strategy to the next plan:
  either migrate the YOLO plugin library as part of Phase 3 and register it
  during detector initialization, or explicitly require plugin-free engine
  plans and document the engine rebuild path.

### R-002 `The advertised Detection2DArray interface has no producer in this phase`

- Severity: HIGH
- Section: `Goals / Architecture / API Surface`
- Type: Spec Alignment
- Problem:
  G-7 says the perception output is `vision_msgs::msg::Detection2DArray`, but
  the architecture and API surface are entirely ROS-free after capture:
  `Detector::detect()` returns `std::vector<Detection>`, `DetectionFilter`
  consumes `std::vector<Detection>`, and there is no node, adapter, or public
  conversion API that produces a `Detection2DArray`.
- Why it matters:
  This is currently an unimplementable contract inside the stated phase
  boundary. A later mission node could publish the message, but then G-7 does
  not belong to Phase 3 as written.
- Recommendation:
  Pick one boundary and state it explicitly in round 01:
  either Phase 3 exports a ROS-facing adapter such as
  `to_detection2d_array(...)` or a lightweight perception node/publisher, or
  G-7 is deferred to a later integration phase and the Phase-3 detector stays
  purely library-shaped.

### R-003 `CUDA/TensorRT build strategy is based on deprecated CMake behavior`

- Severity: HIGH
- Section: `Constraints / Step 7`
- Type: Maintainability
- Problem:
  C-3 and Step 7 propose `find_package(CUDA QUIET)` as the gate for optional
  CUDA support. Current CMake guidance is to use first-class CUDA language
  support and `FindCUDAToolkit`; the old `FindCUDA` module has been deprecated
  for years and is removed under newer CMake policy behavior.
- Why it matters:
  The detector build split is one of the core constraints of this phase:
  Jetson must build with CUDA/TensorRT, while Docker and CI must still build the
  package without them. Basing that on deprecated discovery logic makes the
  plan brittle exactly where the phase needs the most build clarity.
- Recommendation:
  Replace the build strategy in the next plan with a modern one:
  detect CUDA with first-class language support, discover toolkit libraries with
  `find_package(CUDAToolkit)`, and make TensorRT discovery a separate explicit
  check or configurable root path.

### R-004 `cv_py contribution is materially underspecified`

- Severity: MEDIUM
- Section: `Summary / Feature Introduce / Data Structure`
- Type: Design Soundness
- Problem:
  The plan says it merges `cv_py/detect.py`, but the actual Python detector
  logic lives in `cv_py/ros2_v8/.../ros_yolo/detect.py`, and its contribution
  is much larger than “class mapping.” It contains dual-model orchestration,
  camera undistortion, `Detection2DArray` construction, state-aware filtering,
  temporary H-target retention, and mission-coupled servo logic.
- Why it matters:
  Without an explicit kept/deferred/discarded matrix, the phase boundary is
  underspecified. The implementation team will be forced to guess which Python
  behaviors are part of perception and which are intentionally deferred to
  mission code.
- Recommendation:
  Add a source-triage table in round 01 for `cv_py`:
  what is kept in Phase 3, what is deferred to Phase 6, and what is dropped.
  The detector path especially needs a clear decision on dual-model selection,
  undistortion ownership, and message construction.

### R-005 `Validation is too weak for geometry and detector boundary changes`

- Severity: MEDIUM
- Section: `Validation / Acceptance Mapping`
- Type: Validation
- Problem:
  The plan defers unit tests and then validates key behaviors like camera-model
  round trips and transform preservation by “code review.” That is too weak for
  a phase that is extracting complex geometry from `CameraGimbal.h`, changing
  the detector boundary, and removing global state from clustering.
- Why it matters:
  This phase is the first place where subtle math and coordinate errors can be
  introduced without obvious compile failures. A successful `colcon build` does
  not prove that pixel/world transforms or class-selection behavior remain
  correct.
- Recommendation:
  Add a minimal deterministic validation layer in round 01:
  fixture-based geometry checks for at least one pixel/world round trip, one
  detection-ordering/filtering case, and one clustering smoke case with known
  inputs and expected outputs.

---

## Trade-off Advice

### TR-1 `Prefer an explicit adapter boundary for ROS outputs`

- Related Plan Item: `G-7 / API Surface`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Need More Justification
- Advice:
  Keep the internal pipeline ROS-free, but make the ROS-facing message boundary
  explicit instead of implicit.
- Rationale:
  The plan is right to avoid ROS message churn inside the detector path, but
  external consumers still need a stable ROS contract. A thin adapter is a
  better compromise than quietly mixing ROS messages back into the detector core
  or postponing the issue without changing the goal.
- Required Action:
  In the next plan, either add a clear adapter API or remove G-7 from this
  phase and defer it explicitly.

### TR-2 `Prefer a staged detector migration over a single big merge`

- Related Plan Item: `Step 3 / Step 4`
- Topic: Flexibility vs Safety
- Reviewer Position: Prefer Option B
- Advice:
  Treat detector runtime extraction and cv_py behavior selection as two
  separate sub-problems in the plan, even if they are implemented in the same
  phase.
- Rationale:
  The current detector path spans TensorRT runtime concerns, model-selection
  policy, message construction, and mission-coupled logic. Splitting those
  concerns conceptually will reduce the chance of phase creep and make the
  review surface more stable.
- Required Action:
  Reframe the next plan so the TensorRT runtime boundary and the higher-level
  selection/publication behavior are specified independently.

---

## Positive Notes

- The plan correctly identifies `CameraGimbal` and `clustering` as the worst
  current coupling points in the legacy perception path.
- Choosing a library-shaped perception stack first is a good fit for the later
  single-process Jetson architecture.
- The direct-capture decision is consistent with the roadmap and the current
  on-device deployment direction.

---

## Approval Conditions

### Must Fix
- R-001
- R-002
- R-003

### Should Improve
- R-004
- R-005

### Trade-off Responses Required
- TR-1
- TR-2

### Ready for Implementation
- No
- Reason: The detector runtime and build contract are still incomplete, so the
  phase is not yet implementable without avoidable integration risk.
