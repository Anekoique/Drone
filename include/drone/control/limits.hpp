// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace drone::control
{

/// Kinematic limits for position control (speed and acceleration caps).
struct Limits
{
  float speed_max_xy = 2.0f;   ///< Max horizontal speed (m/s)
  float speed_max_z = 1.0f;    ///< Max vertical speed (m/s)
  float speed_max_yaw = 0.3f;  ///< Max yaw rate (rad/s)
  float accel_max_xy = 1.2f;   ///< Max horizontal acceleration (m/s^2)
  float accel_max_z = 1.2f;    ///< Max vertical acceleration (m/s^2)
  float accel_max_yaw = 0.0f;  ///< Max yaw acceleration (rad/s^2)
};

/// Load kinematic limits from a YAML node. Missing keys use defaults.
/// @param yaml YAML node containing limit keys.
/// @return Populated Limits struct.
inline Limits load_limits(const YAML::Node & yaml)
{
  Limits limits;
  if (!yaml) return limits;
  if (yaml["speed_max_xy"]) limits.speed_max_xy = yaml["speed_max_xy"].as<float>();
  if (yaml["speed_max_z"]) limits.speed_max_z = yaml["speed_max_z"].as<float>();
  if (yaml["speed_max_yaw"]) limits.speed_max_yaw = yaml["speed_max_yaw"].as<float>();
  if (yaml["accel_max_xy"]) limits.accel_max_xy = yaml["accel_max_xy"].as<float>();
  if (yaml["accel_max_z"]) limits.accel_max_z = yaml["accel_max_z"].as<float>();
  if (yaml["accel_max_yaw"]) limits.accel_max_yaw = yaml["accel_max_yaw"].as<float>();
  return limits;
}

}  // namespace drone::control
