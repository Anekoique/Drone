// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file frame_transforms.cpp
/// @brief 2D rotation helpers for converting between compass, world (ENU),
///        start-frame, and local (body) coordinate systems.

#include "drone/mission/frame_transforms.hpp"

#include <cmath>

namespace drone::mission
{

Eigen::Vector2f rotate_xy(float x, float y, float angle)
{
  float c = std::cos(angle);
  float s = std::sin(angle);
  return {x * c - y * s, x * s + y * c};
}

Eigen::Vector2f compass_to_world(float x, float y, float heading_rad)
{
  // Legacy rotate_stand2global: rotate by +heading
  return rotate_xy(x, y, heading_rad);
}

Eigen::Vector2f world_to_compass(float x, float y, float heading_rad)
{
  // Legacy rotate_global2stand: rotate by -heading
  return rotate_xy(x, y, -heading_rad);
}

Eigen::Vector2f world_to_start(float x, float y, float start_yaw)
{
  // Legacy rotate_world2start: rotate by start yaw
  return rotate_xy(x, y, start_yaw);
}

Eigen::Vector2f world_to_local(float x, float y, float current_yaw)
{
  // Legacy rotate_world2local: rotate by current yaw
  return rotate_xy(x, y, current_yaw);
}

Eigen::Vector2f local_to_world(float x, float y, float current_yaw)
{
  // Legacy rotate_local2world: rotate by -current yaw
  return rotate_xy(x, y, -current_yaw);
}

Eigen::Vector2f zone_origin_to_world(float dx, float dy, float heading_rad, float /*start_yaw*/)
{
  // Legacy: rotate_global2stand(dx, dy) then send_start_setpoint_command.
  // The zone origin dx/dy are compass-relative offsets.
  // compass_to_world converts them to the world (ENU) frame.
  // start_yaw is not applied here because the result is already in world frame;
  // the start_yaw correction is handled by PosControl which operates in world frame.
  return compass_to_world(dx, dy, heading_rad);
}

}  // namespace drone::mission
