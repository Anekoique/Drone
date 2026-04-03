// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/mission/target_tracker.hpp"
#include "drone/mission/waypoint_nav.hpp"
#include "drone/utils/timer.hpp"

#include <Eigen/Core>

namespace drone::mission
{

/// Handles the airdrop (ball-drop) phase of the mission.
class AirdropHandler
{
public:
  AirdropHandler(Subsystems & subs, const MissionConfig & config);

  /// Set the target tracker (must be called before execute).
  void set_tracker(TargetTracker * tracker) { tracker_ = tracker; }

  /// Navigate to shot zone. Returns airdrop when timeout (12s default).
  /// @param zone_origin Zone origin in world frame.
  /// @param start_yaw Startup yaw for coordinate transforms.
  FlyState goto_zone(const Eigen::Vector2f & zone_origin, float start_yaw);

  /// Execute airdrop sequence with 5 sub-states.
  /// @param zone_origin Zone origin in world frame.
  /// @param heading_rad Heading for coordinate transforms.
  /// @param start_yaw Startup yaw.
  /// @param fast_mode If true, skip waypoint scanning and go straight to airdrop.
  /// @return goto_recon when all shots complete.
  FlyState execute(
    const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw, bool fast_mode);

  /// Reset handler state for reuse.
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
  TargetTracker * tracker_ = nullptr;
  SubState sub_state_ = SubState::init;
  int shot_counter_ = 1;    ///< Current shot number (1-based)
  int circle_counter_ = 0;  ///< Number of circle targets dropped on
  WaypointState wp_state_;
  Timer state_timer_;
  Timer goto_timer_;
  bool goto_initialized_ = false;
};

}  // namespace drone::mission
