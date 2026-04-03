# `Drivers` PLAN `00`

> Status: Draft
> Feature: `drivers`
> Iteration: `00`
> Owner: Executor
> Depends on:
> - Previous Plan: `none`
> - Review: `none`
> - Master Directive: `none`

---

## Summary

Migrate the MAVROS driver layer: ROS 2 node base, motor control, inertial
navigation, servo controller, and gimbal controller. These modules wrap
MAVROS topics and services into clean C++ interfaces consumed by the
position controller (Phase 5) and mission logic (Phase 6).

## Log

[**Feature Introduce**]

Migrates 5 modules (~1170 lines) from legacy `src/offboard_control/`.
Key modernization: replace raw `OffboardControl_Base*` back-pointers
with `rclcpp::Node::SharedPtr`, make InertialNav fields private with
const getters, merge cv_py servo logic.

[**Review Adjustments**]

N/A — first iteration.

[**Master Compliance**]

N/A — first iteration.

### Changes from Previous Round

[**Added**]
Everything — initial plan.

[**Changed**]
N/A

[**Removed**]
N/A

[**Unresolved**]
N/A

### Response Matrix

| Source | ID | Decision | Resolution |
|--------|----|----------|------------|
| — | — | — | First iteration, no prior findings |

---

## Spec

[**Goals**]

- G-1: Migrate `OffboardControl_Base` → `NodeBase` (ROS 2 node base class).
- G-2: Migrate `Motors` → `Motors` (arm, mode switch, takeoff, land, params).
- G-3: Migrate `InertialNav` → `InertialNav` (odometry, GPS, IMU, rangefinder
  subscribers). Fields private, const getters.
- G-4: Migrate `ServoController` → `Servo` (payload release via MAVROS
  command). Merge cv_py `servo_controller.py` fire_servo logic.
- G-5: Migrate `CameraGimbal` → `Gimbal` (mount control publisher/subscriber).
- G-6: All code compiles as `drone_drivers` library, C++20, zero warnings.
- G-7: No dependency on `OffboardControl.h` — drivers are standalone.

Non-goals:
- NG-1: No position control logic (Phase 5).
- NG-2: No mission state machine (Phase 6).
- NG-3: No unit tests requiring live MAVROS (integration tests deferred).

[**Architecture**]

```
include/drone/drivers/
├── node_base.hpp       ROS 2 node base with parameters and mode switching
├── motors.hpp          Arm, takeoff, land, mode switch, param set
├── inertial_nav.hpp    Odometry, GPS, IMU, rangefinder subscribers
├── servo.hpp           Servo control via MAV_CMD_DO_SET_SERVO
└── gimbal.hpp          Camera gimbal mount control

src/drivers/
├── node_base.cpp
├── motors.cpp
├── inertial_nav.cpp
├── servo.cpp
└── gimbal.cpp
```

Dependency: `drivers → math (Eigen types) + utils (readyaml, timer)`

All driver modules receive `rclcpp::Node::SharedPtr` (not raw pointer)
for creating subscribers, publishers, and service clients.

[**Invariants**]

- I-1: No `OffboardControl_Base*` raw back-pointers. Use `rclcpp::Node::SharedPtr`.
- I-2: InertialNav fields are private. Access via const getters.
- I-3: No `using namespace` in headers.
- I-4: No Chinese comments — English only.
- I-5: Motors state machine preserves legacy takeoff sequence exactly.
- I-6: Servo merges both legacy C++ `ServoController` and cv_py `fire_servo`.

[**Data Structure**]

| Legacy File | Target | Lines | Changes |
|---|---|---|---|
| `OffboardControl_Base.h/cpp` | `node_base.hpp/cpp` | ~167 | Remove commented code, SharedPtr |
| `Motors.h/cpp` | `motors.hpp/cpp` | ~609 | Remove raw ptr, clean state machine |
| `InertialNav.h/cpp` | `inertial_nav.hpp/cpp` | ~247 | Private fields, const getters |
| `ServoController.h` | `servo.hpp/cpp` | ~84 | Split .h/.cpp, merge cv_py fire_servo |
| `CameraGimbal.h` (CameraGimbal class) | `gimbal.hpp/cpp` | ~55 | Extract from Camera, ROS pub/sub only |

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

protected:
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
};

class Motors {
public:
  Motors(rclcpp::Node::SharedPtr node, const std::string& mavros_ns);

  bool takeoff(float current_z, float altitude = 5.0f, float yaw = 0.0f);
  void arm(bool arm);
  void switch_mode(const std::string& mode);
  void set_home_position(float yaw = 0.0f);
  void command_takeoff_or_land(const std::string& cmd, float alt, float yaw);
  void set_param(const std::string& name, double value);

  bool armed() const;
  bool connected() const;
  bool guided() const;
  std::string mode() const;
  uint8_t system_status() const;
  Eigen::Vector3f home_position() const;
};

class InertialNav {
public:
  InertialNav(rclcpp::Node::SharedPtr node, const std::string& mavros_ns);

  Eigen::Vector3f position() const;
  Eigen::Vector4f velocity() const;
  Eigen::Quaternionf orientation() const;
  Eigen::Vector3f gps() const;
  float altitude() const;
  float rangefinder_height() const;
  float yaw() const;
};

class Servo {
public:
  Servo(rclcpp::Node::SharedPtr node, const std::string& mavros_ns);
  void set_servo(int servo_number, float pwm_position);
  void fire_servo(int servo_id, float open_pwm = 1864, float close_pwm = 1200);
};

class Gimbal {
public:
  Gimbal(rclcpp::Node::SharedPtr node, const std::string& mavros_ns);
  void set_gimbal(float pitch, float roll, float yaw);
  double gimbal_pitch() const;
  double gimbal_roll() const;
  double gimbal_yaw() const;
};

}  // namespace drone
```

[**Constraints**]

- C-1: All MAVROS topic/service names are parameterized via `mavros_ns`.
- C-2: Motors takeoff state machine must match legacy sequence exactly
  (GUIDED → arm → set_home → takeoff command).
- C-3: QoS profiles must use `rmw_qos_profile_sensor_data` for subscribers
  (matching legacy behavior).
- C-4: Servo `fire_servo` must match cv_py behavior (open → delay → close).

---

## Implement

### Execution Flow

[**Main Flow**]

1. Create directory structure.
2. Migrate NodeBase (extract from OffboardControl_Base).
3. Migrate Motors (clean state machine, SharedPtr).
4. Migrate InertialNav (private fields, const getters).
5. Migrate Servo (merge C++ + cv_py).
6. Migrate Gimbal (extract from CameraGimbal).
7. Update CMakeLists.txt — `drone_drivers` library.
8. Update package.xml — mavros_msgs dependency.
9. Format + Docker build + test.

[**Failure Flow**]

1. Missing MAVROS deps → add to package.xml/Dockerfile.

[**State Transition**]

Not applicable — driver modules are stateful at runtime but migration
is a compile-time task.

### Implementation Plan

[**Step 1 — NodeBase**]

From `OffboardControl_Base.h/cpp`:
- ROS 2 node with sim_mode/debug_mode/print_info/fast_mode parameters.
- Mode switch client.
- Elapsed time tracking.
- Remove all commented-out code.
- Remove static `start` member — use instance member.

[**Step 2 — Motors**]

From `Motors.h/cpp`:
- MAVROS service clients: arm, takeoff, land, set_home, set_mode, param_set.
- State subscriber (armed, connected, guided, mode, system_status).
- Home position subscriber.
- Takeoff state machine (init → guided → arm → takeoff → end).
- Replace `OffboardControl_Base*` with `rclcpp::Node::SharedPtr`.
- Replace `new Timer()` with `drone::Timer` member.

[**Step 3 — InertialNav**]

From `InertialNav.h/cpp`:
- MAVROS topic subscribers: odom, GPS, altitude, IMU, rangefinder.
- All fields private with const getter methods.
- `yaw()` computed from quaternion (same formula as legacy).
- Remove commented-out getters.

[**Step 4 — Servo**]

From `ServoController.h` + `cv_py/servo_controller.py`:
- `set_servo(id, pwm)`: send MAV_CMD_DO_SET_SERVO via MAVROS command service.
- `fire_servo(id)`: open → wait → close sequence (from cv_py).
- Async service calls with callbacks.

[**Step 5 — Gimbal**]

From `CameraGimbal.h` (CameraGimbal class only, Camera already in Phase 3):
- Publisher to `/mavros/mount_control/command`.
- Subscriber to `/mavros/mount_control/status`.
- `set_gimbal(pitch, roll, yaw)`.
- Current angle getters.

[**Step 6 — CMakeLists.txt**]

```cmake
add_library(drone_drivers
  src/drivers/node_base.cpp
  src/drivers/motors.cpp
  src/drivers/inertial_nav.cpp
  src/drivers/servo.cpp
  src/drivers/gimbal.cpp
)
target_link_libraries(drone_drivers PUBLIC drone_utils)
ament_target_dependencies(drone_drivers PUBLIC
  rclcpp mavros_msgs geometry_msgs nav_msgs sensor_msgs)
```

[**Step 7 — package.xml**]

Add: `mavros_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`.

## Trade-offs

- T-1: **`rclcpp::Node::SharedPtr` vs inherit from `rclcpp::Node`**
  - SharedPtr composition: drivers don't need to be nodes themselves.
    They use the node's services/subscriptions.
  - Inherit: simpler but forces every driver to be a full node.
  - Decision: SharedPtr composition. Only `NodeBase` inherits Node.
    Motors/InertialNav/Servo/Gimbal receive a SharedPtr.

- T-2: **Keep Motors takeoff state machine vs simplify**
  - Keep: exact legacy behavior, proven in competition.
  - Simplify: cleaner but risks changing flight behavior.
  - Decision: Keep. Mechanical transfer of state machine.

---

## Validation

[**Unit Tests**]

- V-UT-1: NodeBase constructs with default params (mock node).
- V-UT-2: InertialNav getters return default values before any callback.

[**Integration Tests**]

- V-IT-1: `colcon build` succeeds.
- V-IT-2: `colcon test` passes.
- V-IT-3: All driver headers compile standalone.

[**Edge Case Validation**]

- V-E-1: `grep -r "OffboardControl" include/drone/drivers/` → zero.
- V-E-2: `grep -r "using namespace" include/drone/drivers/` → zero.
- V-E-3: No public mutable fields in InertialNav.

[**Acceptance Mapping**]

| Goal / Constraint | Validation |
|-------------------|------------|
| G-1 | V-IT-1, V-UT-1 |
| G-2 | V-IT-1, motors compiles |
| G-3 | V-E-3, V-UT-2 |
| G-4 | V-IT-1, servo compiles |
| G-5 | V-IT-1, gimbal compiles |
| G-6 | V-IT-1 (zero warnings) |
| G-7 | V-E-1 |
| C-1 | mavros_ns parameter in all constructors |
| C-2 | Code review of takeoff state machine |
| C-3 | QoS profile in subscriber setup |
| C-4 | fire_servo has open/close sequence |
