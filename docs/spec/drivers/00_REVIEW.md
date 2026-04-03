# `Drivers` REVIEW `00`

> Status: Open
> Feature: `drivers`
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
- Blocking Issues: `4`
- Non-Blocking Issues: `4`



## Summary

The plan is structurally sound. The five-module decomposition is the right
split, the dependency direction (drivers depends on utils, not perception)
matches the roadmap, and the decision to use SharedPtr composition for
Motors/InertialNav/Servo/Gimbal while only NodeBase inherits rclcpp::Node
is a defensible ROS 2 pattern.

However, there are four blocking issues. First, NodeBase inherits
rclcpp::Node but also hands out its own SharedPtr to child drivers --
this creates a shared_from_this hazard that the plan does not address.
Second, the Gimbal publisher uses mavros_msgs::msg::MountControl, which
was deprecated in MAVROS 2.x in favor of
mavros_msgs::msg::GimbalManagerSetPitchYaw; the plan should explicitly
document which MAVROS version is targeted and confirm the message type.
Third, the plan omits mavros_msgs::srv::ParamSetV2 from the package.xml
dependency discussion and the CMake ament_target_dependencies, despite
DEPENDENCIES.md listing it as a Motors service. Fourth, the
fire_servo open-wait-close sequence uses a blocking delay in what will be
a single-threaded executor context, which can starve MAVROS heartbeat
callbacks and trigger a failsafe.

Non-blocking issues cover missing thread-safety documentation for
InertialNav callback fields, incomplete validation for the takeoff state
machine, a missing std_msgs dependency, and the absence of any error
handling strategy for MAVROS service call failures.



---

## Findings

### R-001 `NodeBase shared_from_this hazard when handing SharedPtr to child drivers`

- Severity: CRITICAL
- Section: `Architecture / API Surface`
- Type: Correctness
- Problem:
  NodeBase inherits from rclcpp::Node and is the single ROS 2 node. The
  plan says Motors, InertialNav, Servo, and Gimbal each receive an
  `rclcpp::Node::SharedPtr`. But the plan does not specify how that
  SharedPtr is obtained. If NodeBase is constructed on the stack or via
  make_unique, calling shared_from_this() is undefined behavior. If it is
  constructed via make_shared, the plan must document that requirement as
  an invariant and show the construction site.

  More critically, the API surface shows `NodeBase` as a concrete class
  the user constructs, then separately constructs Motors/InertialNav/etc
  with a SharedPtr. This means either: (a) NodeBase must always be held
  in a shared_ptr, which is a hidden precondition not documented in any
  invariant, or (b) the child drivers must receive a raw pointer or
  reference instead of SharedPtr, which contradicts I-1.
- Why it matters:
  This is the central composition pattern for the entire driver layer and
  all downstream phases (control, mission). Getting the ownership model
  wrong here propagates to every module that touches MAVROS.
- Recommendation:
  Resolve the ownership model explicitly. Options:
  1. NodeBase provides a factory: `static shared_ptr<NodeBase> create(...)`,
     and children receive the result. Document that stack construction is
     prohibited.
  2. Children receive `rclcpp::Node&` (reference) instead of SharedPtr.
     This is simpler and avoids the shared_from_this problem entirely.
     The node outlives its children by construction.
  3. NodeBase is not a node itself -- it is a plain class that holds a
     `rclcpp::Node::SharedPtr` obtained externally (from the top-level
     main or component container). This is the cleanest composition but
     requires changing the inheritance model.

  Pick one and add it as an invariant in round 01.



### R-002 `MountControl message type is deprecated in MAVROS 2.x`

- Severity: HIGH
- Section: `Architecture / Step 5 / API Surface`
- Type: Correctness
- Problem:
  The plan specifies that Gimbal publishes to `/mavros/mount_control/command`
  using `mavros_msgs::msg::MountControl`. In MAVROS 2.x for ROS 2 Humble,
  the mount control interface was reworked. The legacy MountControl message
  may still exist for backward compatibility, but the canonical interface
  uses `mavros_msgs::msg::GimbalManagerSetPitchYaw` or the
  `mavros_msgs::srv::CommandLong` MAV_CMD_DO_MOUNT_CONTROL path.

  DEPENDENCIES.md lists MountControl as a known message type, but that
  document may itself be carrying forward a legacy assumption.
- Why it matters:
  If the target MAVROS version does not publish/subscribe on the expected
  topics, the gimbal module will silently do nothing at runtime -- no
  compile error, no obvious failure. This is especially dangerous because
  NG-3 defers integration testing.
- Recommendation:
  In round 01, explicitly document the MAVROS version (apt package version
  on Humble) and verify the mount_control topic/message availability. If
  the project is on MAVROS 2.7+, migrate to the current gimbal plugin
  interface. If it must support an older MAVROS, document that constraint.



### R-003 `fire_servo blocking delay in single-threaded executor`

- Severity: HIGH
- Section: `Constraints / C-4 / Step 4`
- Type: Correctness
- Problem:
  C-4 requires fire_servo to perform an open-wait-close sequence matching
  cv_py behavior. The Python version uses `rospy.sleep()` or
  `time.sleep()` between open and close. The plan does not specify how the
  C++ version implements this delay.

  In a single-threaded rclcpp executor (which the Jetson single-process
  architecture implies), a blocking sleep inside fire_servo will block the
  entire executor. This means no MAVROS state callbacks, no InertialNav
  updates, and no heartbeat processing for the duration of the servo
  delay. Depending on the delay length, this can trigger an ArduPilot
  GCS failsafe (no heartbeat for >5s) or a GUIDED mode timeout.
- Why it matters:
  This is a flight-safety issue. A blocked executor during payload release
  means the drone is flying blind with no position feedback.
- Recommendation:
  Specify the delay mechanism in round 01. Options:
  1. Use a ROS 2 timer callback (non-blocking): fire_servo sets the open
     PWM, creates a one-shot timer, and the timer callback sets the close
     PWM.
  2. Use async service calls with a state machine (open_sent -> wait ->
     close_sent).
  3. Document that fire_servo must be called from a multi-threaded
     executor or a separate callback group.

  Option 1 is the simplest and matches ROS 2 idioms.



### R-004 `Missing mavros_msgs service types in package.xml and CMake`

- Severity: HIGH
- Section: `Step 6 / Step 7`
- Type: Spec Alignment
- Problem:
  The CMake snippet in Step 6 lists `mavros_msgs` as an ament dependency,
  which is correct at the package level. However, the plan does not call
  out that `mavros_msgs` must be present in package.xml as a `<depend>`
  (Step 7 only says "Add: mavros_msgs, geometry_msgs, nav_msgs,
  sensor_msgs").

  More importantly, the plan references `ParamSetV2` (from DEPENDENCIES.md)
  but MAVROS on Humble may ship `ParamSet` (v1) or `ParamSetV2` depending
  on the exact version. The plan does not specify which service type is
  used in the Motors `set_param` implementation.
- Why it matters:
  A mismatch between the expected service type and the installed MAVROS
  version will cause a compile failure that is not obvious from the plan
  text. Since the plan defers integration testing, this would only surface
  during implementation.
- Recommendation:
  In round 01:
  1. Confirm the exact MAVROS service type for param/set (ParamSet vs
     ParamSetV2) by checking the installed mavros_msgs package.
  2. Add mavros_msgs explicitly to the package.xml snippet.
  3. Add std_msgs to the dependency list -- State.msg and other mavros_msgs
     types transitively depend on it, and explicit is better than implicit.



### R-005 `InertialNav thread safety for callback-mutated fields`

- Severity: MEDIUM
- Section: `Architecture / Invariants / API Surface`
- Type: Correctness
- Problem:
  InertialNav subscribes to 5 MAVROS topics. Each callback updates private
  fields. The const getters expose those fields to the caller (which runs
  in the same or different callback context). The plan does not specify
  any synchronization strategy.

  In a single-threaded executor this is safe by construction, but the plan
  does not document that assumption. If a future phase switches to a
  multi-threaded executor or a separate callback group (which R-003 may
  require), InertialNav becomes a data race.
- Why it matters:
  The Eigen types used (Vector3f, Vector4f, Quaternionf) are not atomic.
  A torn read on a partially-written quaternion produces invalid
  orientation data, which feeds directly into the position controller.
- Recommendation:
  Add an invariant or design note in round 01: either (a) document that
  InertialNav requires a single-threaded executor as a precondition, or
  (b) add a mutex/atomic strategy for the mutable fields behind the
  const getters.



### R-006 `No error handling strategy for MAVROS service call failures`

- Severity: MEDIUM
- Section: `API Surface / Execution Flow / Failure Flow`
- Type: Correctness
- Problem:
  The failure flow section is a single line: "Missing MAVROS deps -> add to
  package.xml/Dockerfile." This covers build-time failures only.

  At runtime, every MAVROS service call (arm, set_mode, takeoff, land,
  set_home, command, param_set) can fail: the service may not be
  available, the call may time out, or the flight controller may reject
  the command. The plan does not specify return types, error reporting,
  or retry behavior for any of these.

  The API surface shows `void arm(bool)`, `void switch_mode(...)`, etc.
  These void returns give the caller no way to know if the operation
  succeeded.
- Why it matters:
  The Motors takeoff state machine (GUIDED -> arm -> set_home -> takeoff)
  is a strict sequence. If any step fails silently, the next step will
  also fail or produce undefined flight behavior. This is a safety issue.
- Recommendation:
  In round 01:
  1. Change arm/switch_mode/set_home/command_takeoff_or_land to return
     bool or std::expected indicating success/failure.
  2. Add a service call timeout constant (e.g., 5s) as a configurable
     parameter.
  3. Document the retry policy for the takeoff state machine (retry N
     times? abort? fallback to land?).



### R-007 `Takeoff state machine validation is insufficient`

- Severity: MEDIUM
- Section: `Validation / Acceptance Mapping`
- Type: Validation
- Problem:
  I-5 says "Motors state machine preserves legacy takeoff sequence exactly."
  C-2 says "Motors takeoff state machine must match legacy sequence exactly."
  The acceptance mapping for C-2 is "Code review of takeoff state machine."

  Code review alone is insufficient for a flight-critical state machine.
  The legacy Motors.cpp is ~609 lines with a multi-state takeoff sequence
  (init -> guided -> arm -> set_home -> takeoff -> complete). Each
  transition depends on MAVROS state callbacks. A manual review cannot
  reliably verify that every transition guard, timeout, and edge case is
  preserved.
- Why it matters:
  A missed or reordered state transition in the takeoff sequence can
  result in arming without GUIDED mode (flyaway) or sending takeoff
  before arming (rejected command with no retry).
- Recommendation:
  Add unit tests for the takeoff state machine in round 01:
  1. Mock the MAVROS state subscriber to inject state transitions.
  2. Verify that takeoff() drives through the correct sequence.
  3. Verify that takeoff() handles out-of-order callbacks gracefully.
  4. These tests do not require live MAVROS -- they only need a mock
     node that publishes State messages.



### R-008 `velocity() returns Vector4f -- semantics unclear`

- Severity: MEDIUM
- Section: `API Surface`
- Type: API
- Problem:
  `InertialNav::velocity()` returns `Eigen::Vector4f`. A velocity vector
  is typically 3D (vx, vy, vz). The fourth component is not documented.
  If it represents the velocity magnitude, yaw rate, or a homogeneous
  coordinate, that should be explicit in the API. If it is an artifact of
  the legacy code, it should be corrected to Vector3f.
- Why it matters:
  Every consumer of velocity() must know what the fourth element means.
  An undocumented fourth component in a flight-critical value invites
  misuse.
- Recommendation:
  In round 01, either:
  1. Change to Vector3f if the fourth component is unused, or
  2. Document the fourth component's semantics in the API surface and
     consider using a named struct instead of a raw Eigen vector.



---

## Trade-off Advice

### TR-1 `SharedPtr composition vs Node reference`

- Related Plan Item: `T-1`
- Topic: Flexibility vs Safety
- Reviewer Position: Prefer Option B (reference)
- Advice:
  Use `rclcpp::Node&` (const reference) for child drivers instead of
  `rclcpp::Node::SharedPtr`.
- Rationale:
  The plan correctly identifies that drivers should not be nodes themselves.
  However, SharedPtr composition introduces the shared_from_this hazard
  described in R-001 and forces a specific ownership model on the
  top-level node. A reference avoids this entirely: NodeBase owns its
  lifetime, children borrow it. This is also more consistent with the
  immutability principle -- a reference cannot be reseated, making the
  dependency relationship static and clear.

  The only case where SharedPtr is required is if a driver needs to
  outlive its parent node, which should never happen in this architecture.

  Note: some rclcpp APIs (create_subscription, create_publisher) require
  a Node pointer/reference, not a SharedPtr. Using a reference naturally
  fits these APIs.
- Required Action:
  Evaluate whether any rclcpp API in the driver layer truly requires
  SharedPtr (not just Node*). If none do, switch to `rclcpp::Node&` and
  simplify the ownership model. If some do, document which APIs and why.



### TR-2 `Keep Motors takeoff state machine exactly vs modernize`

- Related Plan Item: `T-2`
- Topic: Compatibility vs Clean Design
- Reviewer Position: Prefer Option A (keep exact), with conditions
- Advice:
  Keep the exact legacy state machine as proposed, but add the unit tests
  specified in R-007 as a precondition for marking the migration complete.
- Rationale:
  The state machine is flight-tested in competition. Changing it
  introduces risk with no immediate benefit. However, "exact mechanical
  transfer" without tests is still risky because the surrounding code
  (SharedPtr, timer, callbacks) is changing. The tests provide the safety
  net that makes the mechanical transfer trustworthy.
- Required Action:
  Keep as-is. Add takeoff state machine unit tests to the validation
  section.



---

## Positive Notes

- The five-module split (NodeBase, Motors, InertialNav, Servo, Gimbal) is
  clean and matches the MAVROS interface boundaries well.
- The decision to keep NodeBase as the only rclcpp::Node subclass and use
  composition for the rest is the right ROS 2 pattern for a single-process
  architecture.
- Merging ServoController (C++) and servo_controller.py (Python) into a
  single Servo class eliminates a real source of behavioral divergence in
  the legacy codebase.
- The invariant set (I-1 through I-6) is well-chosen and directly
  addresses the worst legacy problems.
- The dependency direction (drivers -> utils, not drivers -> perception)
  is correct and matches the roadmap dependency graph.

---

## Approval Conditions

### Must Fix
- R-001
- R-003
- R-004
- R-006

### Should Improve
- R-002
- R-005
- R-007
- R-008

### Trade-off Responses Required
- TR-1
- TR-2

### Ready for Implementation
- No
- Reason: The node ownership model (R-001) is unresolved and affects every
  driver constructor signature. The blocking delay in fire_servo (R-003)
  is a flight-safety issue. These must be resolved before implementation
  can begin safely.
