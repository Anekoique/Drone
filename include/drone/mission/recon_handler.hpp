// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/mission/waypoint_nav.hpp"
#include "drone/utils/timer.hpp"

#include <Eigen/Core>

namespace drone::mission
{

/// Handles the reconnaissance patrol phase of the mission.
class ReconHandler
{
public:
  ReconHandler(Subsystems & subs, const MissionConfig & config);

  /// Navigate to recon zone. Returns recon_patrol when timeout (7.5s).
  /// @param zone_origin Zone origin in world frame.
  /// @param start_yaw Startup yaw for coordinate transforms.
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Patrol recon zone waypoints. Returns landing when complete.
  /// @param zone_origin Zone origin in world frame.
  /// @param heading_rad Heading for coordinate transforms.
  /// @param start_yaw Startup yaw.
  FlyState patrol(const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw);

  /// Reset handler state for reuse.
  void reset();

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  WaypointState wp_state_;
  Timer goto_timer_;
  bool goto_initialized_ = false;
};

}  // namespace drone::mission
