#pragma once

#include "drone/mission/frame_transforms.hpp"
#include "drone/mission/mission_types.hpp"
#include "drone/utils/timer.hpp"

namespace drone::mission
{

struct WaypointState
{
  int index = 0;
  bool initialized = false;
  Timer timer;

  void reset()
  {
    index = 0;
    initialized = false;
    timer = Timer{};
  }
};

/// Navigate through zone waypoints using PosControl.
/// Converts normalized 0-1 waypoints to world coordinates using zone dimensions and origin.
/// Returns true when all waypoints have been visited.
bool waypoint_goto_next(
  Subsystems & subs, const ZoneConfig & zone, const Eigen::Vector2f & zone_origin_world,
  float heading_rad, float start_yaw, WaypointState & state, float timeout_per_wp,
  float accuracy = 0.2f);

}  // namespace drone::mission
