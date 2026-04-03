#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/mission/waypoint_nav.hpp"
#include "drone/utils/timer.hpp"

#include <Eigen/Core>

namespace drone::mission
{

class AirdropHandler
{
public:
  AirdropHandler(Subsystems & subs, const MissionConfig & config);

  /// Navigate to shot zone. Returns airdrop when timeout (12s default).
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Execute airdrop sequence with 5 sub-states.
  /// Returns goto_recon when complete.
  FlyState execute(
    const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw, bool fast_mode);

  void reset();

private:
  enum class SubState
  {
    init,
    world_approach,
    pixel_approach,
    wait,
    end
  };

  Subsystems & subs_;
  const MissionConfig & config_;
  SubState sub_state_ = SubState::init;
  int shot_counter_ = 1;
  int circle_counter_ = 0;
  WaypointState wp_state_;
  Timer state_timer_;
  Timer goto_timer_;
  bool goto_initialized_ = false;
};

}  // namespace drone::mission
