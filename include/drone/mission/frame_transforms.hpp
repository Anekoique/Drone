#pragma once

#include <Eigen/Core>

namespace drone::mission
{

/// Rotate (x,y) by angle radians (counter-clockwise).
Eigen::Vector2f rotate_xy(float x, float y, float angle);

/// Compass-heading frame to world frame (ENU).
/// heading_rad: compass or real heading in radians.
Eigen::Vector2f compass_to_world(float x, float y, float heading_rad);

/// World frame to compass-heading frame.
Eigen::Vector2f world_to_compass(float x, float y, float heading_rad);

/// World frame to startup-relative frame.
Eigen::Vector2f world_to_start(float x, float y, float start_yaw);

/// World frame to body-local frame.
Eigen::Vector2f world_to_local(float x, float y, float current_yaw);

/// Body-local frame to world frame (inverse of world_to_local).
Eigen::Vector2f local_to_world(float x, float y, float current_yaw);

/// Convert zone config dx/dy from compass frame to world frame
/// with start yaw adjustment.
Eigen::Vector2f zone_origin_to_world(float dx, float dy, float heading_rad, float start_yaw);

}  // namespace drone::mission
