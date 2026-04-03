// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file waypoint_nav.cpp
/// @brief Sequential waypoint navigation within a zone. Converts normalized
///        waypoint coordinates to world frame and advances on a per-waypoint timer.

#include "drone/mission/waypoint_nav.hpp"

#include <cmath>

namespace drone::mission
{

/// Advance to the next waypoint in the zone pattern. Each waypoint is specified
/// as normalized (0-1) coordinates scaled by zone length/width, rotated by
/// compass heading, and offset from the zone origin. Uses time-based transitions.
bool waypoint_goto_next(
  Subsystems & subs, const ZoneConfig & zone, const Eigen::Vector2f & zone_origin_world,
  float heading_rad, float /*start_yaw*/, WaypointState & state, float timeout_per_wp,
  float accuracy)
{
  if (zone.waypoints.empty()) return true;

  if (!state.initialized) {
    state.timer = Timer{};
    state.initialized = true;
  }

  int wp_count = static_cast<int>(zone.waypoints.size());
  if (state.index >= wp_count) return true;

  // Convert normalized waypoint to zone-relative meters
  const auto & wp = zone.waypoints[state.index];
  float local_x = wp.x() * zone.length;
  float local_y = wp.y() * zone.width;

  // Convert to world coordinates: zone origin + compass-rotated local offset
  auto world_offset = compass_to_world(local_x, local_y, heading_rad);

  float target_x = zone_origin_world.x() + world_offset.x();
  float target_y = zone_origin_world.y() + world_offset.y();

  // Send position command
  Eigen::Vector4f target(target_x, target_y, zone.altitude, 0);
  subs.pos_control.go_to_position(target, accuracy);

  // Check arrival by timeout (legacy behavior)
  if (state.timer.elapsed() > timeout_per_wp) {
    state.index++;
    state.timer = Timer{};

    if (state.index >= wp_count) {
      state.initialized = false;
      return true;
    }
  }

  return false;
}

}  // namespace drone::mission
