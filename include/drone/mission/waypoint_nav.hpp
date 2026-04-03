// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/mission/frame_transforms.hpp"
#include "drone/mission/mission_types.hpp"
#include "drone/utils/timer.hpp"

namespace drone::mission
{

/// Tracks progress through a waypoint sequence.
struct WaypointState
{
  int index = 0;  ///< Current waypoint index
  bool initialized = false;
  Timer timer;  ///< Per-waypoint timeout timer

  void reset()
  {
    index = 0;
    initialized = false;
    timer = Timer{};
  }
};

/// Navigate through zone waypoints using PosControl.
/// Converts normalized 0-1 waypoints to world coordinates using zone dimensions and origin.
/// @param subs Hardware subsystem references.
/// @param zone Zone configuration with waypoints.
/// @param zone_origin_world Zone origin in world frame (ENU).
/// @param heading_rad Heading for coordinate transforms.
/// @param start_yaw Startup yaw for coordinate transforms.
/// @param state Mutable navigation state (tracks current waypoint).
/// @param timeout_per_wp Maximum seconds per waypoint before skipping.
/// @param accuracy Distance threshold for waypoint acceptance (m).
/// @return true when all waypoints have been visited.
bool waypoint_goto_next(
  Subsystems & subs, const ZoneConfig & zone, const Eigen::Vector2f & zone_origin_world,
  float heading_rad, float start_yaw, WaypointState & state, float timeout_per_wp,
  float accuracy = 0.2f);

}  // namespace drone::mission
