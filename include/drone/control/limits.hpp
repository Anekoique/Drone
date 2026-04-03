#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>

namespace drone::control
{

struct Limits
{
  float speed_max_xy = 2.0f;
  float speed_max_z = 1.0f;
  float speed_max_yaw = 0.3f;
  float accel_max_xy = 1.2f;
  float accel_max_z = 1.2f;
  float accel_max_yaw = 0.0f;
};

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
