// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file recon_handler.cpp
/// @brief Reconnaissance phase: fly to the recon zone and patrol through
///        its waypoint list. Transitions to landing when all waypoints are visited.

#include "drone/mission/recon_handler.hpp"

#include "drone/mission/frame_transforms.hpp"

namespace drone::mission
{

ReconHandler::ReconHandler(Subsystems & subs, const MissionConfig & config)
: subs_(subs), config_(config)
{
}

FlyState ReconHandler::goto_zone(const Eigen::Vector2f & zone_origin, float /*start_yaw*/)
{
  if (!goto_initialized_) {
    goto_timer_ = Timer{};
    goto_initialized_ = true;
  }

  Eigen::Vector4f target(zone_origin.x(), zone_origin.y(), config_.recon_zone.altitude, 0);
  subs_.pos_control.go_to_position(target, 0.2f);

  // Time-based arrival (legacy: 7.5 seconds)
  if (goto_timer_.elapsed() > 7.5) {
    goto_initialized_ = false;
    return FlyState::recon_patrol;
  }

  return FlyState::goto_recon;
}

FlyState ReconHandler::patrol(
  const Eigen::Vector2f & zone_origin, float heading_rad, float start_yaw)
{
  bool done = waypoint_goto_next(
    subs_, config_.recon_zone, zone_origin, heading_rad, start_yaw, wp_state_, 3.5f);

  return done ? FlyState::landing : FlyState::recon_patrol;
}

void ReconHandler::reset()
{
  wp_state_.reset();
  goto_initialized_ = false;
}

}  // namespace drone::mission
