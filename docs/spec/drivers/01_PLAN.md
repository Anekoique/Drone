# `Drivers` PLAN `01`

> Status: Draft
> Feature: `drivers`
> Iteration: `01`
> Owner: Executor
> Depends on:
> - Previous Plan: `00_PLAN.md`
> - Review: `00_REVIEW.md`
> - Master Directive: `none`

---

## Summary

Revised driver plan addressing all 4 blocking issues: ownership model,
fire_servo non-blocking, MAVROS service types, and error handling.

## Log

[**Review Adjustments**]

- R-001 (CRITICAL): Fixed. Children receive `rclcpp::Node&` reference, not
  SharedPtr. NodeBase outlives children by construction. No shared_from_this.
- R-002 (HIGH): Noted. MountControl message retained — legacy MAVROS on our
  Jetson uses this type. Added version note as comment.
- R-003 (HIGH): Fixed. fire_servo uses ROS 2 one-shot timer (non-blocking).
  No executor starvation.
- R-004 (HIGH): Fixed. Use `mavros_msgs::srv::ParamSetV2` (matches legacy
  CMakeLists.txt). All deps explicit in package.xml.
- R-005 (MEDIUM): Documented. Single-threaded executor is a precondition.
  Added invariant I-7.
- R-006 (MEDIUM): Fixed. Service wrappers return bool. Takeoff state machine
  retries on failure.
- R-007 (MEDIUM): Noted. Mock-based tests deferred — need MAVROS mock infra.
- R-008 (MEDIUM): Fixed. velocity() returns Vector4f where .w() is yaw rate.
  Documented.

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| Review | R-001 | Accepted | Reference, not SharedPtr |
| Review | R-002 | Noted | MountControl retained, version documented |
| Review | R-003 | Accepted | One-shot timer for fire_servo |
| Review | R-004 | Accepted | ParamSetV2 confirmed, deps explicit |
| Review | R-005 | Accepted | Single-threaded invariant added |
| Review | R-006 | Accepted | Bool returns, retry in takeoff |
| Review | R-007 | Noted | Mock tests deferred |
| Review | R-008 | Accepted | Vector4f w() = yaw rate, documented |

---

## Spec

[**Goals**]

Same as round 00 (G-1 through G-7).

[**Architecture**]

Same directory structure. Key change: children receive `rclcpp::Node&`.

```
// Ownership model:
// main.cpp creates NodeBase via make_shared (for executor).
// Children receive Node& reference — no shared_from_this needed.
auto node = std::make_shared<drone::NodeBase>("drone_node", "/mavros/");
drone::Motors motors(*node, "/mavros/");
drone::InertialNav inav(*node, "/mavros/");
drone::Servo servo(*node, "/mavros/");
drone::Gimbal gimbal(*node, "/mavros/");
executor.add_node(node);
```

[**Invariants**]

- I-1: Children receive `rclcpp::Node&` (not SharedPtr, not raw ptr).
- I-2: InertialNav fields private, const getters.
- I-3: No `using namespace` in headers.
- I-4: English comments only.
- I-5: Motors takeoff state machine matches legacy sequence.
- I-6: Servo fire_servo is non-blocking (one-shot timer).
- I-7: All driver callbacks assume single-threaded executor.
- I-8: Service wrappers return bool indicating success/failure.

[**API Surface**]

```cpp
namespace drone {

class NodeBase : public rclcpp::Node {
public:
  explicit NodeBase(const std::string& name, const std::string& mavros_ns = "/mavros/");
  double elapsed_time() const;
  double start_time() const;
  void set_start_time(double t);
  bool sim_mode() const;
  bool debug_mode() const;
  bool fast_mode() const;
};

class Motors {
public:
  Motors(rclcpp::Node& node, const std::string& mavros_ns);
  bool takeoff(float current_z, float altitude = 5.0f, float yaw = 0.0f);
  bool arm(bool arm);
  bool switch_mode(const std::string& mode);
  bool set_home_position(float yaw = 0.0f);
  bool command_takeoff_or_land(const std::string& cmd, float alt, float yaw);
  bool set_param(const std::string& name, double value);

  bool armed() const;
  bool connected() const;
  bool guided() const;
  std::string mode() const;
  uint8_t system_status() const;
  Eigen::Vector3f home_position() const;
};

class InertialNav {
public:
  InertialNav(rclcpp::Node& node, const std::string& mavros_ns);
  Eigen::Vector3f position() const;
  Eigen::Vector4f velocity() const;   // .w() = yaw rate
  Eigen::Quaternionf orientation() const;
  Eigen::Vector3f gps() const;
  float altitude() const;
  float rangefinder_height() const;
  float yaw() const;   // computed from quaternion
};

class Servo {
public:
  Servo(rclcpp::Node& node, const std::string& mavros_ns);
  bool set_servo(int servo_number, float pwm);
  // Non-blocking: opens servo, then closes after delay via one-shot timer.
  void fire_servo(int servo_id, float open_pwm = 1864, float close_pwm = 1200,
                  double delay_sec = 0.5);
};

class Gimbal {
public:
  Gimbal(rclcpp::Node& node, const std::string& mavros_ns);
  void set_gimbal(float pitch, float roll, float yaw);
  // Note: uses mavros_msgs::msg::MountControl (legacy MAVROS).
  // For MAVROS 2.7+, may need migration to GimbalManager plugin.
  double gimbal_pitch() const;
  double gimbal_roll() const;
  double gimbal_yaw() const;
};

}  // namespace drone
```

[**Constraints**]

- C-1: MAVROS namespace parameterized.
- C-2: Takeoff state machine matches legacy.
- C-3: QoS: `rmw_qos_profile_sensor_data` for subscribers.
- C-4: fire_servo non-blocking (one-shot timer, not sleep).
- C-5: Service call timeout: 5 seconds.
- C-6: Uses `mavros_msgs::srv::ParamSetV2`.

---

## Implement

Same steps as round 00, with these changes:

[**Step 1 — NodeBase**]
- `rclcpp::Node` inheritance.
- Mode client created in constructor.
- No shared_from_this usage.

[**Step 2 — Motors**]
- Receives `rclcpp::Node&`.
- Creates service clients via `node.create_client<...>()`.
- Service wrappers return `bool`.
- Takeoff retries arm on timeout (1s timer, matching legacy).

[**Step 4 — Servo**]
- `fire_servo` stores timer as member to prevent premature destruction.
- Per-servo timer map handles concurrent fire_servo calls for different IDs.
```cpp
// Member: std::unordered_map<int, rclcpp::TimerBase::SharedPtr> pending_timers_;

void Servo::fire_servo(int id, float open, float close, double delay) {
  set_servo(id, open);
  // Store timer as member — prevents destruction before callback fires
  pending_timers_[id] = node_.create_wall_timer(
    std::chrono::duration<double>(delay),
    [this, id, close]() {
      set_servo(id, close);
      pending_timers_.erase(id);
    });
}
```

[**Step 6 — CMakeLists.txt**]
```cmake
add_library(drone_drivers
  src/drivers/node_base.cpp
  src/drivers/motors.cpp
  src/drivers/inertial_nav.cpp
  src/drivers/servo.cpp
  src/drivers/gimbal.cpp
)
target_include_directories(drone_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_drivers PUBLIC drone_utils)
ament_target_dependencies(drone_drivers PUBLIC
  rclcpp mavros_msgs geometry_msgs nav_msgs sensor_msgs std_msgs)
```

[**Step 7 — package.xml**]
Add: `mavros_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`.

## Trade-offs

- T-1: **Node& reference vs SharedPtr** — Reference. Simpler, no
  shared_from_this hazard. Node outlives children by construction.
- T-2: **Keep Motors state machine** — Keep. Battle-tested.
- T-3: **fire_servo: one-shot timer vs async state machine** — One-shot
  timer. Simpler, matches ROS 2 idioms, non-blocking.

---

## Validation

Same as round 00, plus:
- V-UT-3: Servo fire_servo does not block (verify timer-based).
- V-E-4: All service wrappers return bool.
- V-E-5: No `shared_from_this` calls in driver code.
