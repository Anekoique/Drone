#pragma once

namespace drone::control
{

/// Normalize angle to [-pi, pi]
float normalize_yaw(float yaw);

/// Shortest angular difference from current to target
float yaw_error(float current, float target);

}  // namespace drone::control
