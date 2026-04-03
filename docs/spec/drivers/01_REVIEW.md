# `Drivers` REVIEW `01`

> Status: Open
> Feature: `drivers`
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

The revised plan successfully resolves three of the four blocking issues
from round 00. The ownership model (R-001) is clean: `Node&` reference
eliminates the shared_from_this hazard entirely, and rclcpp's
create_subscription/create_client/create_publisher are all member
functions of rclcpp::Node that work correctly when called through a
reference. The service wrappers now return bool (R-006), ParamSetV2 is
confirmed (R-004), and the single-threaded executor invariant is
documented (R-005). The response matrix is thorough and every round 00
finding has a clear disposition.

However, the fire_servo one-shot timer implementation (R-003) has a
lifetime bug that makes it blocking in practice. The timer local variable
goes out of scope at the end of fire_servo, destroying the timer before
its callback fires. This must be fixed before implementation.

Two non-blocking issues remain: the timer capture semantics need
clarification, and the deferred mock testing should have at least a
tracking issue or milestone target.



---

## Findings

### R-001 `fire_servo timer destroyed before callback fires`

- Severity: CRITICAL
- Section: `Implement / Step 4`
- Type: Correctness
- Problem:
  The fire_servo code snippet creates a `rclcpp::TimerBase::SharedPtr`
  as a local variable:

  ```cpp
  void Servo::fire_servo(int id, float open, float close, double delay) {
    set_servo(id, open);
    auto timer = node_.create_wall_timer(
      std::chrono::duration<double>(delay),
      [this, id, close, &timer]() {
        set_servo(id, close);
        timer->cancel();
      });
  }
  ```

  When fire_servo returns, `timer` goes out of scope. Since `timer` is
  the only SharedPtr holding the timer, the timer is destroyed. The
  callback will never fire. The servo opens but never closes.

  Additionally, the lambda captures `timer` by reference (`&timer`),
  but `timer` is a local variable. Even if the timer survived (e.g.,
  if rclcpp internally holds a reference), the lambda would hold a
  dangling reference to a destroyed local, invoking undefined behavior
  on `timer->cancel()`.
- Why it matters:
  This is the same class of bug as round 00's R-003 (fire_servo
  correctness), just manifested differently. A servo that opens but
  never closes is a payload-loss hazard. The dangling reference is UB
  that may crash or corrupt memory.
- Recommendation:
  Store the timer as a member variable of Servo:

  ```cpp
  class Servo {
    rclcpp::TimerBase::SharedPtr fire_timer_;
  public:
    void fire_servo(int id, float open, float close, double delay) {
      set_servo(id, open);
      fire_timer_ = node_.create_wall_timer(
        std::chrono::duration<double>(delay),
        [this, id, close]() {
          set_servo(id, close);
          fire_timer_->cancel();
          fire_timer_.reset();
        });
    }
  };
  ```

  This keeps the timer alive until the callback runs, and the lambda
  captures `this` (which is stable) rather than a local reference.
  The `reset()` releases the timer after use. Since I-7 guarantees a
  single-threaded executor, no mutex is needed.

  Note: if fire_servo can be called again before the previous timer
  fires, the old timer is replaced and its callback never runs. This
  is acceptable if the design intent is "last call wins," but it
  should be documented as an invariant or handled with a queue.



### R-002 `Concurrent fire_servo calls silently drop previous close`

- Severity: MEDIUM
- Section: `API Surface / Constraints`
- Type: Correctness
- Problem:
  If fire_servo is called twice in rapid succession (e.g., for two
  different servo IDs), the second call would overwrite the member
  timer, canceling the first close. This could leave a servo in the
  open position.
- Why it matters:
  In a multi-servo payload configuration, simultaneous release commands
  are plausible. The plan does not document whether concurrent
  fire_servo calls are supported.
- Recommendation:
  Either:
  1. Use a `std::unordered_map<int, rclcpp::TimerBase::SharedPtr>`
     keyed by servo ID, so each servo has its own timer.
  2. Document that fire_servo is single-servo-at-a-time and add a
     precondition check (log warning if timer is active).



### R-003 `Deferred mock tests lack a milestone target`

- Severity: LOW
- Section: `Validation`
- Type: Validation
- Problem:
  R-007 from round 00 (takeoff state machine unit tests) was deferred
  with "need MAVROS mock infra." The round 01 plan notes this as
  "Noted" but does not specify when the mock infrastructure will be
  built or which iteration will add the tests.
- Why it matters:
  Without a tracking mechanism, deferred tests tend to stay deferred
  indefinitely. The takeoff state machine is the highest-risk
  component in the driver layer and remains unvalidated beyond code
  review.
- Recommendation:
  Add a concrete target: "Mock MAVROS infrastructure and takeoff state
  machine tests will be added in iteration 02 or 03, before the
  control layer begins integration." This creates accountability
  without blocking the current round.



---

## Round 00 Blocking Issue Resolution

| Round 00 ID | Issue | Status | Notes |
|-------------|-------|--------|-------|
| R-001 | shared_from_this hazard | Resolved | Node& reference eliminates the hazard. rclcpp member functions (create_subscription, create_client, create_publisher, create_wall_timer) are callable on a reference. No SharedPtr needed. |
| R-003 | fire_servo blocking delay | Partially Resolved | One-shot timer is the correct approach, but the implementation has a timer lifetime bug (see this round's R-001). The design intent is correct; the code needs fixing. |
| R-004 | ParamSetV2 ambiguity | Resolved | ParamSetV2 confirmed, package.xml dependencies explicit, std_msgs added. |
| R-006 | Missing error handling | Resolved | Service wrappers return bool. Takeoff retries arm on timeout. |

---

## Trade-off Advice

### TR-1 `One-shot timer vs member timer for fire_servo`

- Related Plan Item: `T-3`
- Topic: Simplicity vs Correctness
- Reviewer Position: Prefer member timer
- Advice:
  Store the timer as a Servo member variable rather than a local.
- Rationale:
  A local timer is destroyed on function return. The member timer
  pattern is equally simple (one extra member variable) and is the
  standard ROS 2 idiom for deferred one-shot work. The rclcpp
  documentation and examples consistently show timers stored as
  members precisely because the executor only holds a weak reference to
  the timer -- the owning SharedPtr must outlive the callback.
- Required Action:
  Adopt the member timer pattern as shown in R-001's recommendation.



---

## Positive Notes

- The Node& reference pattern is the cleanest resolution of the round 00
  ownership debate. It is simpler than a factory method, avoids
  shared_from_this entirely, and naturally fits rclcpp's API since
  create_subscription et al. are member functions callable on a reference.
- The response matrix in the plan's Log section is exemplary -- every
  round 00 finding has a clear disposition with rationale.
- Adding I-7 (single-threaded executor invariant) and I-8 (bool returns)
  as explicit invariants strengthens the contract for downstream phases.
- The API surface is clean, consistent, and well-documented. The Vector4f
  w() = yaw rate clarification resolves a real source of confusion.
- The ownership model diagram in the Architecture section makes the
  construction order and lifetime guarantees immediately obvious.

---

## Approval Conditions

### Must Fix
- R-001

### Should Improve
- R-002
- R-003

### Trade-off Responses Required
- TR-1

### Ready for Implementation
- No
- Reason: The fire_servo timer lifetime bug (R-001) means the close
  callback will never fire. This is a one-line fix (local to member) but
  must be reflected in the plan before implementation begins. Once R-001
  is fixed, the plan is ready for implementation.
