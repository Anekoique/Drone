// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Core>

namespace drone::mission
{

/// Rotate (x, y) by angle radians (counter-clockwise).
/// @param x X coordinate.
/// @param y Y coordinate.
/// @param angle Rotation angle in radians.
/// @return Rotated (x, y).
Eigen::Vector2f rotate_xy(float x, float y, float angle);

/// Compass-heading frame to world frame (ENU).
/// @param x X in compass frame.
/// @param y Y in compass frame.
/// @param heading_rad Compass or real heading in radians.
/// @return (x, y) in world frame.
Eigen::Vector2f compass_to_world(float x, float y, float heading_rad);

/// World frame to compass-heading frame.
/// @param x X in world frame.
/// @param y Y in world frame.
/// @param heading_rad Heading in radians.
/// @return (x, y) in compass frame.
Eigen::Vector2f world_to_compass(float x, float y, float heading_rad);

/// World frame to startup-relative frame.
/// @param x X in world frame.
/// @param y Y in world frame.
/// @param start_yaw Startup yaw in radians.
/// @return (x, y) in startup-relative frame.
Eigen::Vector2f world_to_start(float x, float y, float start_yaw);

/// World frame to body-local frame.
/// @param x X in world frame.
/// @param y Y in world frame.
/// @param current_yaw Current heading in radians.
/// @return (x, y) in body-local frame.
Eigen::Vector2f world_to_local(float x, float y, float current_yaw);

/// Body-local frame to world frame (inverse of world_to_local).
/// @param x X in body-local frame.
/// @param y Y in body-local frame.
/// @param current_yaw Current heading in radians.
/// @return (x, y) in world frame.
Eigen::Vector2f local_to_world(float x, float y, float current_yaw);

/// Convert zone config dx/dy from compass frame to world frame
/// with start yaw adjustment.
/// @param dx Zone X offset in compass frame.
/// @param dy Zone Y offset in compass frame.
/// @param heading_rad Compass heading in radians.
/// @param start_yaw Startup yaw in radians.
/// @return Zone origin (x, y) in world frame.
Eigen::Vector2f zone_origin_to_world(float dx, float dy, float heading_rad, float start_yaw);

}  // namespace drone::mission
