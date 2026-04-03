#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/mission/waypoint_nav.hpp"
#include "drone/utils/timer.hpp"

#include <Eigen/Core>

namespace drone::mission
{

class ReconHandler
{
public:
  ReconHandler(Subsystems & subs, const MissionConfig & config);

  /// Navigate to recon zone. Returns recon_patrol when timeout (7.5s).
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Patrol recon zone. Returns landing when complete.
  FlyState patrol(const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw);

  void reset();

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  WaypointState wp_state_;
  Timer goto_timer_;
  bool goto_initialized_ = false;
};

}  // namespace drone::mission
