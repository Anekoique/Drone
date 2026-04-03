# `mission` PLAN `01`

> Status: Draft
> Feature: `mission`
> Iteration: `01`
> Owner: Planner
> Dependencies:
>
> - Phase 4 drivers: NodeBase, Motors, InertialNav, Servo, Gimbal (complete)
> - Phase 5 control: PosControl, VelocityController, MavrosCommander (complete)
> - Phase 3 perception: CameraModel, DetectionFilter, Clustering (complete)
> - Round 00 Review: `00_REVIEW.md`

---

## Log

### Review Adjustments

Round 00 was blocked with 7 HIGH, 2 MEDIUM, 2 LOW findings.

### Response Matrix

| ID | Severity | Resolution |
|---|---|---|
| B-001 | HIGH | Fixed: Added `land_to_start` to FlyState enum. Added `LandingHandler::land_to_start()` method with 3 sub-states. Added triage row and DroneNode dispatch case. |
| B-002 | HIGH | Fixed: Expanded AirdropHandler sub-states to `{init, world_approach, pixel_approach, wait, end}`. Added fast_mode path, circle_counter fallback (12 cycles), double-barrel logic. Documented all timing constants. |
| B-003 | HIGH | Fixed: Added `local_to_world()` and documented that compass transforms take heading_rad parameter (works for both compass and real heading). Added `heading_real_rad` to MissionConfig. |
| B-004 | HIGH | Fixed: Added `shot_zone.altitude_surround`, `bucket_height`. Added `AirdropConfig` (from can_config.yaml) and `LandingConfig` (from land_config.yaml) to MissionConfig. Specified YAML key mapping. |
| B-005 | HIGH | Fixed: DroneNode inherits NodeBase. Added system-status readiness guard and GPS validity check to timer_callback pseudocode. fast_mode accessible via NodeBase::fast_mode(). |
| B-006 | HIGH | Fixed: Added visual_approach sub-state to LandingHandler with surround search pattern (linear +/-3m), stub detector interface, and 0.2 m/s descent velocity command. |
| B-007 | HIGH | Fixed: Added WaypointState::reset() method. Documented reset protocol: DroneNode calls handler.reset() on state entry transitions. Clarified I-3 applies only to frame_transforms.hpp. |
| N-001 | MEDIUM | Fixed: Added timeout_sec parameter to goto_zone() with defaults (12.0s shot, 7.5s recon). Documented altitude field used. |
| N-002 | MEDIUM | Fixed: Added fly_state_to_int() in mission_types.hpp with legacy-compatible encoding. |
| N-003 | LOW | Fixed: Added C-8 constraint. Reset calls shown in timer_callback pseudocode. |
| N-004 | LOW | Fixed: Added mission_types.hpp as the ROADMAP state_machine.hpp equivalent. Noted in non-goals. |

---

## Spec

### Goals

- G-1: Extract each mission phase into a focused handler class (< 400 lines each)
- G-2: Replace template-based state machine with simple enum + dispatch
- G-3: Centralize all frame transforms into a single `frame_transforms` module
- G-4: Create `MissionConfig` to load all YAML configuration (mission, airdrop, landing)
- G-5: Create `DroneNode` as the top-level ROS 2 node composing all subsystems
- G-6: Break the circular dependency between StateMachine and OffboardControl
- G-7: Preserve all legacy mission behavior including LandToStart
- G-8: Make handlers testable (pass subsystem references, no direct ROS in handlers)

### Non-goals

- NG-1: Changing mission logic or flight behavior
- NG-2: Adding new mission phases not present in legacy
- NG-3: Integrating TensorRT detector (Phase 7, airdrop/landing use stub)
- NG-4: Debug modes (MYPID, Print_Info, Termial_Control, Reflush_config)
- NG-5: PAL statistics publishing
- NG-6: Auto-tune (deferred since Phase 5)
- NG-7: ROADMAP lists `state_machine.hpp`; `mission_types.hpp` serves this role

### Architecture

```
include/drone/mission/
  mission_types.hpp          # FlyState enum, fly_state_to_int, MissionConfig, Subsystems
  frame_transforms.hpp       # All rotate functions as stateless free functions
  waypoint_nav.hpp           # waypoint_goto_next with WaypointState
  takeoff_handler.hpp        # init + takeoff
  airdrop_handler.hpp        # goto_shot + airdrop (5 sub-states)
  recon_handler.hpp          # goto_recon + patrol
  landing_handler.hpp        # doland (visual approach) + land_to_start
  drone_node.hpp             # Top-level node, state dispatch

src/mission/
  frame_transforms.cpp
  waypoint_nav.cpp
  takeoff_handler.cpp
  airdrop_handler.cpp
  recon_handler.cpp
  landing_handler.cpp
  drone_node.cpp
```

#### CMake Target Layout

```
drone_mission (NEW, links drone_control + drone_drivers + drone_perception)
  All src/mission/*.cpp files
```

### Invariants

- I-1: No handler directly accesses another handler's state
- I-2: All inter-handler communication goes through DroneNode state dispatch
- I-3: Frame transforms in `frame_transforms.hpp` are stateless free functions
- I-4: Handlers receive subsystem references at construction
- I-5: FlyState enum has no dependency on any handler class
- I-6: Mission config loaded once at construction, immutable during flight
- I-7: No circular dependencies: handlers depend on drivers/control, not vice versa

### Data Structure

#### File Migration Triage

| Legacy Element | Action | Target |
|---|---|---|
| `OffboardControl_Base` | Drop | Already migrated as NodeBase (Phase 4) |
| `OffboardControl::FlyState_init()` | Move | `TakeoffHandler::initialize()` |
| `OffboardControl::timer_callback()` | Move | `DroneNode::timer_callback()` |
| `OffboardControl::waypoint_goto_next()` | Move | `waypoint_nav::waypoint_goto_next()` |
| `OffboardControl::catch_target()` | Move | `AirdropHandler::catch_target()` (stub until Phase 7) |
| `OffboardControl::Doshot()` | Move | `AirdropHandler::pixel_approach()` inner sub-function |
| `FlyState::Doshot handler` (StateMachine.cpp) | Move | `AirdropHandler::execute()` with 5 sub-states |
| `OffboardControl::surrounding_shot_area()` | Move | `AirdropHandler::patrol()` via waypoint_nav |
| `OffboardControl::surrounding_scout_area()` | Move | `ReconHandler::patrol()` via waypoint_nav |
| `OffboardControl::Doland()` | Move | `LandingHandler::visual_approach()` |
| `FlyState::Doland handler` (StateMachine.cpp) | Move | `LandingHandler::execute()` with 5 sub-states |
| `FlyState::LandToStart handler` | Move | `LandingHandler::land_to_start()` with 3 sub-states |
| `OffboardControl::read_configs()` | Move | `MissionConfig::load()` |
| `rotate_global2stand()` et al. (7 functions) | Move | `frame_transforms.hpp` free functions |
| `OffboardControl::send_*_setpoint_command()` | Drop | Use PosControl/MavrosCommander directly |
| `OffboardControl::get_x/y/z_pos()` et al. | Drop | Use InertialNav directly |
| `StateMachine` template dispatch | Replace | switch in DroneNode::timer_callback() |
| `FlyState` enum | Move | `mission_types.hpp` |
| `fly_state_to_int()` | Move | `mission_types.hpp` with legacy-compatible encoding |
| Subsystem composition | Move | `Subsystems` struct in `mission_types.hpp` |
| `surround_shot_points[]` | Move | `MissionConfig` shot_zone.waypoints from YAML |
| `surround_see_points[]` | Move | `MissionConfig` recon_zone.waypoints from YAML |
| `DoshotState` sub-state machine | Move | `AirdropHandler` internal state |
| `DolandState` sub-state machine | Move | `LandingHandler` internal state |
| `LandToStartState` sub-state machine | Move | `LandingHandler` internal state |
| `save_log()` | Defer | Phase 7 diagnostics |
| `publish_statistics()` | Drop | NG-5 |
| MYPID, Print_Info, Termial_Control | Drop | NG-4 debug modes |
| `is_first_run_` sentinel | Replace | handler.reset() called by DroneNode on state entry |

#### MissionConfig

```cpp
namespace drone::mission {

struct ZoneConfig {
  float dx = 0;
  float dy = 0;
  float length = 8.0f;
  float width = 5.0f;
  float altitude = 4.5f;
  float altitude_surround = 3.0f;
  float altitude_low = 1.8f;
  std::vector<Eigen::Vector2f> waypoints;
};

struct AirdropConfig {
  PID::Defaults pid;
  control::Limits limits;
  float radius = 0.08f;
  float accuracy = 0.75f;
  float shot_duration = 0.6f;
  float shot_wait = 0.8f;
  float tar_z = 1.0f;
  // Per-servo drop points (left/right relative to camera)
  // YAML keys: shot_target_x_l/y_l/z_l, shot_target_x_r/y_r/z_r
  Eigen::Vector3f shot_point_left = {-0.05f, 0.015f, 0.07f};
  Eigen::Vector3f shot_point_right = {0.05f, 0.015f, 0.07f};
  // Per-servo target pixels
  // YAML keys: tar_x_l/tar_y_l, tar_x_r/tar_y_r (separate scalars)
  Eigen::Vector2f tar_pixel_left = {665, 470};
  Eigen::Vector2f tar_pixel_right = {615, 470};
};

struct LandingConfig {
  PID::Defaults pid;
  control::Limits limits;
  float scout_halt = 2.5f;   // YAML key: scout_halt
  float scout_x = 0;         // YAML key: scout_x
  float scout_y = 0;         // YAML key: scout_y
  float accuracy = 0.00005f; // YAML key: accuracy
  float tar_z = 0.2f;        // YAML key: tar_z
  Eigen::Vector2f tar_pixel = {640, 400};  // YAML keys: tar_x, tar_y (separate scalars)
  float descent_speed = -0.2f;
  float descent_duration = 1.0f;
  float surround_range = 3.0f;
  float surround_step = 1.0f;
};

struct MissionConfig {
  float heading_compass_deg = 180.0f;  // YAML key: headingangle_compass
  float heading_compass_rad = 0;      // = heading_compass_deg * M_PI / 180
  float heading_real_rad = 0;         // YAML key: headingangle_real (degrees, converted)
                                      // Default: heading_compass_rad when key absent
  float default_yaw = 0;
  float bucket_height = 0.3f;

  ZoneConfig shot_zone;
  ZoneConfig recon_zone;
  AirdropConfig airdrop;
  LandingConfig landing;

  float servo_open_pwm = 1980;
  float servo_close_pwm = 1200;
  bool prefer_large_target = true;

  Eigen::Vector3f drone_to_camera = Eigen::Vector3f::Zero();
  float takeoff_altitude = 2.0f;

  static MissionConfig load(const std::string & mission_yaml,
    const std::string & airdrop_yaml, const std::string & landing_yaml);
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
  land_to_start,
  finished,
};

int fly_state_to_int(FlyState state);

struct Subsystems {
  Motors & motors;
  InertialNav & inav;
  control::PosControl & pos_control;
  Servo & servo;
  Gimbal & gimbal;
};

}  // namespace drone::mission

// --- frame_transforms.hpp ---
namespace drone::mission {

Eigen::Vector2f rotate_xy(float x, float y, float angle);
Eigen::Vector2f compass_to_world(float x, float y, float heading_rad);
Eigen::Vector2f world_to_compass(float x, float y, float heading_rad);
Eigen::Vector2f world_to_start(float x, float y, float start_yaw);
Eigen::Vector2f world_to_local(float x, float y, float current_yaw);
Eigen::Vector2f local_to_world(float x, float y, float current_yaw);
Eigen::Vector2f zone_origin_to_world(
  float dx, float dy, float heading_rad, float start_yaw);

}  // namespace drone::mission

// --- waypoint_nav.hpp ---
namespace drone::mission {

struct WaypointState {
  int index = 0;
  bool initialized = false;
  Timer timer;
  void reset();
};

/// Navigate through zone waypoints. Returns true when all visited.
bool waypoint_goto_next(
  Subsystems & subs, const ZoneConfig & zone,
  const Eigen::Vector2f & zone_origin_world,
  float heading_rad, float start_yaw,
  WaypointState & state,
  float timeout_per_wp, float accuracy = 0.2f);

}  // namespace drone::mission

// --- takeoff_handler.hpp ---
namespace drone::mission {

class TakeoffHandler {
public:
  TakeoffHandler(Subsystems & subs, const MissionConfig & config);

  FlyState initialize();
  FlyState takeoff();

  Eigen::Vector4f start_position() const;
  float start_yaw() const;

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

  /// Navigate to shot zone. Returns airdrop when timeout (12s).
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Execute airdrop sequence. Returns goto_recon when complete.
  FlyState execute(
    const Eigen::Vector2f & zone_origin, float heading_rad,
    float start_yaw, bool fast_mode);

  void reset();

private:
  enum class SubState { init, world_approach, pixel_approach, wait, end };

  Subsystems & subs_;
  const MissionConfig & config_;
  SubState sub_state_ = SubState::init;
  int shot_counter_ = 1;
  int circle_counter_ = 0;
  WaypointState wp_state_;
  Timer state_timer_;
  Timer wait_timer_;
  Timer goto_timer_;
  bool goto_initialized_ = false;
};

}  // namespace drone::mission

// --- recon_handler.hpp ---
namespace drone::mission {

class ReconHandler {
public:
  ReconHandler(Subsystems & subs, const MissionConfig & config);

  /// Navigate to recon zone. Returns recon_patrol when timeout (7.5s).
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Patrol recon zone. Returns landing when complete.
  FlyState patrol(
    const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw);

  void reset();

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  WaypointState wp_state_;
  Timer goto_timer_;
  bool goto_initialized_ = false;
};

}  // namespace drone::mission

// --- landing_handler.hpp ---
namespace drone::mission {

class LandingHandler {
public:
  LandingHandler(Subsystems & subs, const MissionConfig & config);

  /// Doland sequence: RTL -> wait -> visual approach -> descent -> LAND.
  FlyState execute(float heading_rad, float start_yaw);

  /// LandToStart: fly to (0,0,2) -> wait 19s -> LAND.
  FlyState land_to_start();

  void reset();
  void reset_land_to_start();

private:
  enum class DolandState { rtl, wait, visual_approach, descent, land };
  enum class LandToStartState { init, wait, land };

  Subsystems & subs_;
  const MissionConfig & config_;

  DolandState doland_state_ = DolandState::rtl;
  LandToStartState lts_state_ = LandToStartState::init;
  Timer state_timer_;

  // Visual approach state (surround_land_ starts at -3 for full ±3m range)
  int surround_land_ = -3;
  Timer target_loss_timer_;
  bool approach_initialized_ = false;
  // reset() must set surround_land_ = -3 to restore full search range
};

}  // namespace drone::mission

// --- drone_node.hpp ---
namespace drone::mission {

class DroneNode : public NodeBase {
public:
  DroneNode(const std::string & mavros_ns, const std::string & config_dir);

private:
  void timer_callback();

  // Subsystem ownership
  Motors motors_;
  InertialNav inav_;
  Servo servo_;
  Gimbal gimbal_;
  control::PosControl pos_control_;
  Subsystems subs_;

  // Mission
  MissionConfig config_;
  FlyState state_ = FlyState::init;
  FlyState prev_state_ = FlyState::init;

  TakeoffHandler takeoff_;
  AirdropHandler airdrop_;
  ReconHandler recon_;
  LandingHandler landing_;

  Eigen::Vector2f shot_origin_world_ = Eigen::Vector2f::Zero();
  Eigen::Vector2f recon_origin_world_ = Eigen::Vector2f::Zero();

  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
};

}  // namespace drone::mission
```

### Constraints

- C-1: Handlers must not create ROS publishers/subscribers
- C-2: Frame transforms in frame_transforms.hpp are stateless free functions
- C-3: Waypoint arrays in MissionConfig are loaded from YAML, not hardcoded
- C-4: DroneNode timer runs at 20 Hz (50ms)
- C-5: State transitions are explicit return values from handlers
- C-6: No shared_ptr for subsystems within handlers, use references
- C-7: Forward-only state transitions in normal flow (init->...->finished)
- C-8: DroneNode must call handler.reset() on first entry into each state. Detect via prev_state_ != state_ after dispatch.

---

## Implement

### Execution Flow

#### DroneNode::timer_callback()

```
timer_callback() @ 20 Hz
  // System readiness guard (from legacy timer_callback lines 37-55)
  if (!motors_.connected() || !motors_.armed()) return;
  if (inav_.position() == Vector3f::Zero() && state_ != FlyState::init) return;

  // State entry reset
  if (state_ != prev_state_) {
    switch (state_) {
      airdrop:      airdrop_.reset(); break;
      recon_patrol: recon_.reset(); break;
      landing:      landing_.reset(); break;
      land_to_start: landing_.reset_land_to_start(); break;
    }
    prev_state_ = state_;
  }

  // State dispatch
  switch (state_) {
    init:          state_ = takeoff_.initialize(); break;
    takeoff:       state_ = takeoff_.takeoff(); break;
    goto_shot:     state_ = airdrop_.goto_zone(shot_origin_, start_yaw); break;
    airdrop:       state_ = airdrop_.execute(shot_origin_, heading, start_yaw, fast_mode()); break;
    goto_recon:    state_ = recon_.goto_zone(recon_origin_, start_yaw); break;
    recon_patrol:  state_ = recon_.patrol(recon_origin_, heading, start_yaw); break;
    landing:       state_ = landing_.execute(heading, start_yaw); break;
    land_to_start: state_ = landing_.land_to_start(); break;
    finished:      /* wait 3s then shutdown */ break;
  }

  // Publish state
  publish_state(fly_state_to_int(state_));
```

#### AirdropHandler::execute() sub-states

```
SubState::init
  ├─ Reset counters: shot_counter=1, circle_counter=0
  ├─ Load airdrop PID/limits into PosControl
  └─ → SubState::world_approach

SubState::world_approach
  ├─ If no clustered targets: patrol via waypoint_goto_next at altitude_surround
  ├─ If targets available:
  │   ├─ Fly to cal_center[counter] at altitude_low
  │   ├─ Apply drone_to_camera offset via local_to_world
  │   ├─ If within accuracy AND timer > 6s: → SubState::pixel_approach
  │   └─ If timer > 10s: force → SubState::pixel_approach
  ├─ fast_mode: fire servo immediately → SubState::wait
  ├─ circle_counter >= 12: fallback to waypoint patrol
  └─ Timeout 70s → SubState::end

SubState::pixel_approach
  ├─ catch_target() stub (Phase 7: visual servo)
  ├─ If Doshot() returns true: fire servo → SubState::wait
  └─ Timeout: → SubState::wait

SubState::wait
  ├─ Wait 2 seconds
  ├─ If shot_counter <= 1: increment, → SubState::world_approach
  └─ Else: → SubState::end

SubState::end
  ├─ Open all servos (11, 12) as safety
  ├─ Wait 2 seconds
  ├─ Restore default PID/limits
  └─ return FlyState::goto_recon
```

#### LandingHandler::execute() sub-states

```
DolandState::rtl
  ├─ motors.switch_mode("RTL")
  └─ → DolandState::wait

DolandState::wait
  ├─ Wait 18 seconds
  ├─ motors.switch_mode("GUIDED")
  └─ → DolandState::visual_approach

DolandState::visual_approach
  ├─ Load landing PID/limits into PosControl
  ├─ Fly to scout position: compass_to_world(scout_x, scout_y+0.3)
  ├─ If H-target not detected for >2s:
  │   ├─ surround_land search: linear ±3m at 1m steps
  │   └─ Fly to compass_to_world(scout_x + offset, scout_y)
  ├─ If H-target detected:
  │   └─ catch_target() stub (Phase 7: visual servo approach)
  ├─ If altitude < tar_z + 0.1: → DolandState::descent
  └─ Timeout 19s: → DolandState::descent

DolandState::descent
  ├─ send_velocity_timed(0, 0, -0.2, 0, 1.0)
  └─ When done: → DolandState::land

DolandState::land
  ├─ motors.switch_mode("LAND")
  └─ return FlyState::finished
```

### Implementation Plan

#### Step 1: Create `mission_types.hpp`

- FlyState enum with 9 states (including land_to_start)
- fly_state_to_int() with legacy-compatible encoding
- Subsystems struct
- MissionConfig, ZoneConfig, AirdropConfig, LandingConfig structs
- MissionConfig::load() reads mission.yaml + airdrop.yaml + landing.yaml

#### Step 2: Create `frame_transforms.hpp/cpp`

- 7 free functions: rotate_xy, compass_to_world, world_to_compass,
  world_to_start, world_to_local, local_to_world, zone_origin_to_world
- All take heading_rad parameter (works for both compass and real heading)

#### Step 3: Create `waypoint_nav.hpp/cpp`

- WaypointState with reset()
- waypoint_goto_next() with timeout_per_wp parameter

#### Step 4: Create `takeoff_handler.hpp/cpp`

- initialize(): capture start position, set home, configure gimbal
- takeoff(): delegate to Motors::takeoff(), monitor altitude

#### Step 5: Create `airdrop_handler.hpp/cpp`

- goto_zone(): time-based arrival (12s timeout)
- execute(): 5 sub-states with fast_mode, circle_counter, double-barrel
- catch_target() stub for Phase 7 detector integration

#### Step 6: Create `recon_handler.hpp/cpp`

- goto_zone(): time-based arrival (7.5s timeout)
- patrol(): waypoint_goto_next through recon zone

#### Step 7: Create `landing_handler.hpp/cpp`

- execute(): 5 sub-states (rtl, wait, visual_approach, descent, land)
- land_to_start(): 3 sub-states (init, wait, land)
- Visual approach with surround search stub (detector in Phase 7)

#### Step 8: Create `drone_node.hpp/cpp`

- Inherits NodeBase
- Owns all subsystems and handlers
- timer_callback with readiness guard and state dispatch
- State entry reset via prev_state_ tracking

#### Step 9: Update CMakeLists.txt

- Add drone_mission target
- Add tests

#### Step 10: Write unit tests

- test_frame_transforms.cpp: all rotation functions, round-trips
- test_mission_config.cpp: YAML loading from all 3 config files
- test_fly_state.cpp: fly_state_to_int encoding matches legacy

### Trade-offs

#### T-1: Return FlyState vs callback pattern
- Decision: Return FlyState. Clean, testable, no indirection.

#### T-2: Detector integration now vs Phase 7
- Decision: Defer. Stubs in airdrop and landing handlers.

#### T-3: Config consolidation now vs Phase 7
- Decision: Keep 3 separate YAML files. MissionConfig::load() takes 3 paths.

#### T-4: Debug modes
- Decision: Drop. NG-4.

---

## Validation

### Unit Tests

- V-UT-1: rotate_xy correctness (0, 90, 180, 270 degrees)
- V-UT-2: compass_to_world / world_to_compass round-trip
- V-UT-3: zone_origin_to_world matches legacy transform chain
- V-UT-4: local_to_world is inverse of world_to_local
- V-UT-5: MissionConfig::load reads all fields from 3 YAML files
- V-UT-6: MissionConfig::load with partial YAML uses defaults
- V-UT-7: fly_state_to_int encoding matches legacy values
- V-UT-8: WaypointState::reset clears index and timer
- V-UT-9: FlyState enum has all 9 states

### Integration Tests

- V-IT-1: All mission headers compile standalone
- V-IT-2: DroneNode constructs with valid config
- V-IT-3: State dispatch covers all FlyState values (no default fallthrough)

### Acceptance Mapping

| Goal | Validation |
|------|------------|
| G-1 Handler extraction | 4 handlers each < 400 lines |
| G-2 Simple state machine | DroneNode switch, no templates |
| G-3 Centralized transforms | V-UT-1..4, single module |
| G-4 MissionConfig | V-UT-5..6, loads 3 YAML files |
| G-5 DroneNode | V-IT-2..3, inherits NodeBase |
| G-6 No circular deps | I-7, CMake target enforced |
| G-7 Preserve behavior | Triage complete, LandToStart included |
| G-8 Testable handlers | Handlers take references, no ROS |
