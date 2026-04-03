# `mission` PLAN `00`

> Status: Draft
> Feature: `mission`
> Iteration: `00`
> Owner: Planner
> Dependencies:
>
> - Phase 4 drivers: NodeBase, Motors, InertialNav, Servo, Gimbal (complete)
> - Phase 5 control: PosControl, VelocityController, MavrosCommander (complete)
> - Phase 3 perception: CameraModel, DetectionFilter, Clustering (complete)

---

## Log

### Feature Introduce

Legacy `OffboardControl` (~672 lines header + large .cpp) is a monolithic "god class"
that bundles: ROS 2 node lifecycle, 7 composed subsystems, state machine orchestration,
5 mission phases (init, takeoff, airdrop, recon, landing), 3 coordinate frame systems,
waypoint patrol logic, target tracking, servo control, and diagnostics.

`StateMachine.h/cpp` uses a template-based state handler pattern with forward pointer
to `OffboardControl`, creating a circular dependency. Each state handler directly
accesses all members of the god class.

Phase 6 breaks this into discrete mission handlers with explicit dependencies,
a clean state machine, centralized frame transforms, and a top-level `DroneNode`
that composes everything.

### Changes from Previous Round

- N/A (first iteration)

---

## Spec

### Goals

- G-1: Extract each mission phase into a focused handler class (< 400 lines each)
- G-2: Replace template-based state machine with simple enum + dispatch
- G-3: Centralize all frame transforms into a single `FrameTransforms` utility
- G-4: Create `MissionConfig` to consolidate all YAML configuration
- G-5: Create `DroneNode` as the top-level ROS 2 node composing all subsystems
- G-6: Break the circular dependency between StateMachine and OffboardControl
- G-7: Preserve all legacy mission behavior (init, takeoff, airdrop, recon, landing)
- G-8: Make handlers testable without ROS (pass subsystem references)

### Non-goals

- NG-1: Changing mission logic or flight behavior
- NG-2: Adding new mission phases not present in legacy
- NG-3: Integrating TensorRT inference (Phase 7)
- NG-4: Migrating debug modes (MYPID, Print_Info, Termial_Control, Reflush_config)
- NG-5: Migrating PAL statistics publishing
- NG-6: Migrating auto-tune (deferred since Phase 5)
- NG-7: Consolidating config files (Phase 7)

### Architecture

```
include/drone/mission/
  mission_types.hpp        # FlyState enum, MissionConfig, Subsystems struct
  frame_transforms.hpp     # All rotate_* functions centralized
  waypoint_nav.hpp         # waypoint_goto_next logic
  takeoff_handler.hpp      # FlyState::init + takeoff
  airdrop_handler.hpp      # Goto_shotpoint + Doshot (sub-states)
  recon_handler.hpp        # Goto_scoutpoint + Surround_see
  landing_handler.hpp      # Doland + LandToStart
  drone_node.hpp           # Top-level ROS 2 node, state dispatch

src/mission/
  frame_transforms.cpp
  waypoint_nav.cpp
  takeoff_handler.cpp
  airdrop_handler.cpp
  recon_handler.cpp
  landing_handler.cpp
  drone_node.cpp

config/
  mission.yaml             # Existing file, already migrated in Phase 0
```

#### CMake Target Layout

```
drone_mission (NEW, has ROS, links drone_control + drone_drivers + drone_perception)
  frame_transforms.cpp       (could be non-ROS, but depends on InertialNav getters)
  waypoint_nav.cpp
  takeoff_handler.cpp
  airdrop_handler.cpp
  recon_handler.cpp
  landing_handler.cpp
  drone_node.cpp
```

### Invariants

- I-1: No handler directly accesses another handler's state
- I-2: All inter-handler communication goes through `DroneNode` state dispatch
- I-3: Frame transforms are stateless free functions (no class state)
- I-4: Handlers receive subsystem references at construction, not pointers
- I-5: `FlyState` enum has no dependency on any handler class
- I-6: Mission config loaded once at construction, immutable during flight
- I-7: No circular dependencies: handlers depend on drivers/control, not vice versa

### Data Structure

#### File Migration Triage

| Legacy Element | Action | Target |
|---|---|---|
| `OffboardControl_Base` | Drop | Already migrated as `NodeBase` in Phase 4 |
| `OffboardControl::FlyState_init()` | Move | `TakeoffHandler::initialize()` |
| `OffboardControl::timer_callback()` | Move | `DroneNode::timer_callback()` |
| `OffboardControl::waypoint_goto_next()` | Move | `waypoint_nav.hpp::waypoint_goto_next()` |
| `OffboardControl::catch_target()` | Move | `AirdropHandler::catch_target()` |
| `OffboardControl::Doshot()` | Move | `AirdropHandler::execute()` |
| `OffboardControl::surrounding_shot_area()` | Move | `AirdropHandler::patrol()` |
| `OffboardControl::surrounding_scout_area()` | Move | `ReconHandler::patrol()` |
| `OffboardControl::Doland()` | Move | `LandingHandler::execute()` |
| `OffboardControl::read_configs()` | Move | `MissionConfig::load()` |
| `rotate_global2stand()` et al. | Move | `frame_transforms.hpp` free functions |
| `OffboardControl::send_*_setpoint_command()` | Drop | Use PosControl/MavrosCommander directly |
| `OffboardControl::get_x/y/z_pos()` et al. | Drop | Use InertialNav directly |
| `StateMachine` template dispatch | Replace | Simple switch in `DroneNode::timer_callback()` |
| `FlyState` enum | Move | `mission_types.hpp` |
| Subsystem composition (7 shared_ptrs) | Move | `Subsystems` struct in `mission_types.hpp` |
| `surround_shot_points[]` | Move | `MissionConfig` YAML arrays |
| `surround_see_points[]` | Move | `MissionConfig` YAML arrays |
| `DoshotState` sub-state machine | Move | `AirdropHandler` internal state |
| `save_log()` | Defer | Phase 7 diagnostics |
| `publish_statistics()` | Drop | NG-5 |
| MYPID, Print_Info, Termial_Control | Drop | NG-4 debug modes |

#### Subsystems Struct

```cpp
namespace drone::mission {

struct Subsystems {
  Motors & motors;
  InertialNav & inav;
  control::PosControl & pos_control;
  Servo & servo;
  Gimbal & gimbal;
  // Perception references added when TensorRT integrates (Phase 7)
};

}  // namespace drone::mission
```

#### MissionConfig

```cpp
namespace drone::mission {

struct ZoneConfig {
  float dx = 0;              // X offset from home (meters)
  float dy = 0;              // Y offset from home (meters)
  float length = 8.0f;       // Zone length (meters)
  float width = 5.0f;        // Zone width (meters)
  float altitude = 4.5f;     // Cruise altitude (meters)
  float altitude_low = 1.8f; // Low-altitude pass (meters)
  std::vector<Eigen::Vector2f> waypoints;  // Normalized 0-1 patrol path
};

struct MissionConfig {
  float heading_compass_deg = 0;    // Compass heading (degrees)
  float heading_real_rad = 0;       // Computed: radians
  float default_yaw = 0;

  ZoneConfig shot_zone;             // Airdrop zone
  ZoneConfig recon_zone;            // Recon zone

  float servo_open_pwm = 1980;
  float servo_close_pwm = 1200;
  bool prefer_large_target = true;

  Eigen::Vector3f drone_to_camera = Eigen::Vector3f::Zero();

  float takeoff_altitude = 2.0f;
  float waypoint_accuracy = 0.2f;

  static MissionConfig load(const std::string & config_path);
};

}  // namespace drone::mission
```

### API Surface

```cpp
// --- mission_types.hpp ---
namespace drone::mission {

enum class FlyState {
  init,
  takeoff,
  goto_shot,
  airdrop,
  goto_recon,
  recon_patrol,
  landing,
  finished,
};

struct Subsystems { /* as above */ };
struct MissionConfig { /* as above */ };

}  // namespace drone::mission

// --- frame_transforms.hpp ---
namespace drone::mission {

/// Rotate (x,y) by angle radians.
Eigen::Vector2f rotate_xy(float x, float y, float angle);

/// World frame (ENU) to compass-heading frame.
Eigen::Vector2f world_to_compass(float x, float y, float heading_rad);

/// Compass-heading frame to world frame.
Eigen::Vector2f compass_to_world(float x, float y, float heading_rad);

/// World frame to startup-relative frame (offset by start yaw).
Eigen::Vector2f world_to_start(float x, float y, float start_yaw);

/// World frame to body-local frame (offset by current yaw).
Eigen::Vector2f world_to_local(float x, float y, float current_yaw);

/// Convert zone config dx/dy from compass frame to world frame.
Eigen::Vector2f zone_origin_to_world(
  float dx, float dy, float heading_rad, float start_yaw);

}  // namespace drone::mission

// --- waypoint_nav.hpp ---
namespace drone::mission {

struct WaypointState {
  int index = 0;
  bool initialized = false;
  Timer timer;
};

/// Navigate through waypoints in a zone. Returns true when all visited.
bool waypoint_goto_next(
  Subsystems & subs, const ZoneConfig & zone,
  const Eigen::Vector2f & zone_origin_world,
  float heading_rad, float start_yaw,
  WaypointState & state,
  float timeout_per_wp = 3.5f, float accuracy = 0.2f);

}  // namespace drone::mission

// --- takeoff_handler.hpp ---
namespace drone::mission {

class TakeoffHandler {
public:
  TakeoffHandler(Subsystems & subs, const MissionConfig & config);

  /// Initialize coordinate frames, capture start position, set home.
  /// Returns next state (takeoff or init if not ready).
  FlyState initialize();

  /// Execute takeoff. Returns goto_shot when altitude reached.
  FlyState takeoff();

  Eigen::Vector4f start_position() const { return start_pos_; }
  float start_yaw() const { return start_pos_.w(); }

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  Eigen::Vector4f start_pos_ = Eigen::Vector4f::Zero();
  bool init_done_ = false;
};

}  // namespace drone::mission

// --- airdrop_handler.hpp ---
namespace drone::mission {

class AirdropHandler {
public:
  AirdropHandler(Subsystems & subs, const MissionConfig & config);

  /// Navigate to shot zone. Returns airdrop when arrived.
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Execute airdrop sequence (sub-state machine).
  /// Returns goto_recon when complete.
  FlyState execute(
    const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw);

  void reset();

private:
  enum class SubState { init, shot, wait, end };

  Subsystems & subs_;
  const MissionConfig & config_;
  SubState sub_state_ = SubState::init;
  int shot_counter_ = 0;
  WaypointState wp_state_;
  Timer state_timer_;
  Timer wait_timer_;
};

}  // namespace drone::mission

// --- recon_handler.hpp ---
namespace drone::mission {

class ReconHandler {
public:
  ReconHandler(Subsystems & subs, const MissionConfig & config);

  /// Navigate to recon zone. Returns recon_patrol when arrived.
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Patrol recon zone. Returns landing when complete.
  FlyState patrol(
    const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw);

  void reset();

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  WaypointState wp_state_;
};

}  // namespace drone::mission

// --- landing_handler.hpp ---
namespace drone::mission {

class LandingHandler {
public:
  LandingHandler(Subsystems & subs, const MissionConfig & config);

  /// Execute landing sequence (RTL -> wait -> GUIDED -> LAND).
  /// Returns finished when landed.
  FlyState execute();

  void reset();

private:
  enum class SubState { rtl, wait, guided, land };

  Subsystems & subs_;
  const MissionConfig & config_;
  SubState sub_state_ = SubState::rtl;
  Timer state_timer_;
};

}  // namespace drone::mission

// --- drone_node.hpp ---
namespace drone::mission {

class DroneNode : public rclcpp::Node {
public:
  DroneNode(const std::string & mavros_ns, const std::string & config_path);

private:
  void timer_callback();

  // Subsystem ownership
  Motors motors_;
  InertialNav inav_;
  Servo servo_;
  Gimbal gimbal_;
  control::PosControl pos_control_;
  Subsystems subs_;

  // Mission state
  MissionConfig config_;
  FlyState state_ = FlyState::init;

  // Handlers
  TakeoffHandler takeoff_;
  AirdropHandler airdrop_;
  ReconHandler recon_;
  LandingHandler landing_;

  // Computed at init
  Eigen::Vector2f shot_origin_world_ = Eigen::Vector2f::Zero();
  Eigen::Vector2f recon_origin_world_ = Eigen::Vector2f::Zero();

  // ROS 2
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
};

}  // namespace drone::mission
```

### Constraints

- C-1: Handlers must not create ROS publishers/subscribers (DroneNode owns all ROS resources)
- C-2: Frame transforms are pure functions, no class state
- C-3: Waypoint arrays in MissionConfig are loaded from YAML, not hardcoded
- C-4: DroneNode timer callback must run at 20 Hz (50ms), matching legacy
- C-5: State transitions are explicit return values, not side effects
- C-6: No shared_ptr for subsystems within handlers, use references
- C-7: Handlers can only transition forward (init->takeoff->shot->recon->land->end)

---

## Implement

### Execution Flow

#### Main Loop: DroneNode::timer_callback()

```
timer_callback() @ 20 Hz
  switch (state_) {
    init:         state_ = takeoff_.initialize();
    takeoff:      state_ = takeoff_.takeoff();
    goto_shot:    state_ = airdrop_.goto_zone(shot_origin_, start_yaw);
    airdrop:      state_ = airdrop_.execute(shot_origin_, heading, start_yaw);
    goto_recon:   state_ = recon_.goto_zone(recon_origin_, start_yaw);
    recon_patrol: state_ = recon_.patrol(recon_origin_, heading, start_yaw);
    landing:      state_ = landing_.execute();
    finished:     shutdown();
  }
  publish state to /drone/state
```

#### Airdrop Sub-state Flow

```
AirdropHandler::execute()
  SubState::init
    ├─ Load airdrop config, reset counters
    └─ → SubState::shot

  SubState::shot
    ├─ Poll detected targets (via subs_.detector when integrated)
    ├─ Cluster targets, sort by preference
    ├─ For current target:
    │   ├─ fly toward using PosControl
    │   ├─ if within accuracy && timer > 6s: fire servo → SubState::wait
    │   └─ if no detection for 12 cycles: patrol waypoints
    └─ Timeout 70s → SubState::end

  SubState::wait
    ├─ Wait 2 seconds (servo settle)
    └─ → SubState::shot (next target) OR SubState::end

  SubState::end
    ├─ Open all servos (safety)
    └─ return FlyState::goto_recon
```

### Implementation Plan

#### Step 1: Create `mission_types.hpp`

- `FlyState` enum (simplified: init, takeoff, goto_shot, airdrop, goto_recon, recon_patrol, landing, finished)
- `Subsystems` struct with references to all drivers/control
- `MissionConfig` struct with `load()` static method
- `ZoneConfig` sub-struct
- File: `include/drone/mission/mission_types.hpp`

#### Step 2: Create `frame_transforms.hpp/cpp`

- Port all `rotate_*()` methods as free functions
- `rotate_xy()`, `world_to_compass()`, `compass_to_world()`, `world_to_start()`, `world_to_local()`
- `zone_origin_to_world()` combines compass + start transforms
- File: `include/drone/mission/frame_transforms.hpp`, `src/mission/frame_transforms.cpp`

#### Step 3: Create `waypoint_nav.hpp/cpp`

- Port `waypoint_goto_next()` with `WaypointState` struct
- Uses PosControl for position commands
- Uses frame_transforms for coordinate conversion
- File: `include/drone/mission/waypoint_nav.hpp`, `src/mission/waypoint_nav.cpp`

#### Step 4: Create `takeoff_handler.hpp/cpp`

- Port `FlyState_init()` to `initialize()`
- Port takeoff logic to `takeoff()`
- Captures start_position for frame transforms
- File: `include/drone/mission/takeoff_handler.hpp`, `src/mission/takeoff_handler.cpp`

#### Step 5: Create `airdrop_handler.hpp/cpp`

- Port `Goto_shotpoint` to `goto_zone()`
- Port `Doshot` with sub-states to `execute()`
- Port `surrounding_shot_area()` to internal patrol
- Port `catch_target()` for target tracking
- Note: detector integration deferred to Phase 7, uses stub for now
- File: `include/drone/mission/airdrop_handler.hpp`, `src/mission/airdrop_handler.cpp`

#### Step 6: Create `recon_handler.hpp/cpp`

- Port `Goto_scoutpoint` to `goto_zone()`
- Port `Surround_see` to `patrol()`
- File: `include/drone/mission/recon_handler.hpp`, `src/mission/recon_handler.cpp`

#### Step 7: Create `landing_handler.hpp/cpp`

- Port `Doland` with sub-states (RTL -> wait -> GUIDED -> LAND)
- File: `include/drone/mission/landing_handler.hpp`, `src/mission/landing_handler.cpp`

#### Step 8: Create `drone_node.hpp/cpp`

- Top-level ROS 2 node inheriting rclcpp::Node
- Owns all subsystems (Motors, InertialNav, Servo, Gimbal, PosControl)
- Owns all handlers
- State dispatch in `timer_callback()`
- Publishes state to topic
- File: `include/drone/mission/drone_node.hpp`, `src/mission/drone_node.cpp`

#### Step 9: Update CMakeLists.txt

- Add `drone_mission` library target linking `drone_control + drone_drivers`
- Add tests

#### Step 10: Write unit tests

- `test_frame_transforms.cpp` — all rotation functions
- `test_waypoint_nav.cpp` — waypoint progression with mock subsystems
- `test_mission_config.cpp` — YAML loading

### Trade-offs

#### T-1: Handler return values vs callback pattern

- Decision: Return `FlyState` from handlers
- Rationale: Explicit, testable, no callback indirection. DroneNode does the dispatch.

#### T-2: Detector integration now vs Phase 7

- Decision: Defer to Phase 7. Airdrop handler has stub interface.
- Rationale: TensorRT detector needs CUDA build. Phase 6 focuses on state machine structure.

#### T-3: Config consolidation now vs Phase 7

- Decision: Defer consolidation to Phase 7. Use existing separate YAML files.
- Rationale: MissionConfig.load() reads from the existing mission.yaml. No config
  file changes needed in Phase 6.

#### T-4: Debug modes (MYPID, Print_Info, etc.)

- Decision: Drop in Phase 6. Can be re-added as optional handlers later.
- Rationale: These are development aids, not competition flight code. Simplifies
  the state machine significantly.

---

## Validation

### Unit Tests

- V-UT-1: `rotate_xy` correctness (0, 90, 180, 270 degrees)
- V-UT-2: `world_to_compass` / `compass_to_world` round-trip
- V-UT-3: `zone_origin_to_world` matches legacy transform chain
- V-UT-4: `MissionConfig::load` reads all fields from YAML
- V-UT-5: `MissionConfig::load` with partial YAML uses defaults
- V-UT-6: `FlyState` enum has correct values
- V-UT-7: `WaypointState` resets properly
- V-UT-8: `LandingHandler` sub-state transitions (rtl->wait->guided->land)

### Integration Tests

- V-IT-1: All mission headers compile standalone
- V-IT-2: `DroneNode` constructs with valid config
- V-IT-3: State dispatch covers all `FlyState` values

### Acceptance Mapping

| Goal | Validation |
|------|------------|
| G-1 Handler extraction | Architecture: 4 handlers each < 400 lines |
| G-2 Simple state machine | DroneNode switch dispatch, no templates |
| G-3 Centralized transforms | V-UT-1..3, single frame_transforms module |
| G-4 MissionConfig | V-UT-4..5 |
| G-5 DroneNode | V-IT-2..3 |
| G-6 No circular deps | I-7 enforced by CMake target |
| G-7 Preserve behavior | Triage table complete, all methods mapped |
| G-8 Testable handlers | V-UT-8, handlers take references |
