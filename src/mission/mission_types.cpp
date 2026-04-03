// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file mission_types.cpp
/// @brief Loads the complete MissionConfig from three YAML files (mission,
///        airdrop, landing). Populates zone waypoints, PID defaults, limits,
///        servo PWM values, and camera-to-drone offsets.

#include "drone/mission/mission_types.hpp"

#include "drone/utils/readyaml.hpp"

#include <cmath>

namespace drone::mission
{

namespace
{

template <typename T>
T yaml_get(const YAML::Node & node, const std::string & key, T default_val)
{
  if (node && node[key]) return node[key].as<T>();
  return default_val;
}

PID::Defaults load_pid(const YAML::Node & node)
{
  PID::Defaults d;
  if (!node) return d;
  d.p = yaml_get(node, "p", 0.0f);
  d.i = yaml_get(node, "i", 0.0f);
  d.d = yaml_get(node, "d", 0.0f);
  d.ff = yaml_get(node, "ff", 0.0f);
  d.dff = yaml_get(node, "dff", 0.0f);
  d.imax = yaml_get(node, "imax", 10.0f);
  return d;
}

control::Limits load_limits(const YAML::Node & node)
{
  control::Limits lim;
  if (!node) return lim;
  lim.speed_max_xy = yaml_get(node, "speed_max_xy", lim.speed_max_xy);
  lim.speed_max_z = yaml_get(node, "speed_max_z", lim.speed_max_z);
  lim.speed_max_yaw = yaml_get(node, "speed_max_yaw", lim.speed_max_yaw);
  lim.accel_max_xy = yaml_get(node, "accel_max_xy", lim.accel_max_xy);
  lim.accel_max_z = yaml_get(node, "accel_max_z", lim.accel_max_z);
  return lim;
}

}  // namespace

MissionConfig MissionConfig::load(
  const std::string & mission_yaml, const std::string & airdrop_yaml,
  const std::string & landing_yaml)
{
  MissionConfig cfg;

  // Mission config
  YAML::Node m = ConfigLoader::load(mission_yaml);
  cfg.heading_compass_deg = yaml_get(m, "headingangle_compass", 180.0f);
  cfg.heading_compass_rad = cfg.heading_compass_deg * static_cast<float>(M_PI) / 180.0f;

  float heading_real_deg = yaml_get(m, "headingangle_real", cfg.heading_compass_deg);
  cfg.heading_real_rad = heading_real_deg * static_cast<float>(M_PI) / 180.0f;

  cfg.shot_zone.dx = yaml_get(m, "dx_shot", 0.0f);
  cfg.shot_zone.dy = yaml_get(m, "dy_shot", 0.0f);
  cfg.shot_zone.altitude = yaml_get(m, "shot_halt", 4.5f);
  cfg.shot_zone.altitude_surround = yaml_get(m, "shot_halt_surround", 3.0f);
  cfg.shot_zone.altitude_low = yaml_get(m, "shot_halt_low", 1.8f);

  cfg.recon_zone.dx = yaml_get(m, "dx_see", 0.0f);
  cfg.recon_zone.dy = yaml_get(m, "dy_see", 0.0f);
  cfg.recon_zone.altitude = yaml_get(m, "see_halt", 3.5f);

  cfg.drone_to_camera.x() = yaml_get(m, "drone_to_camera_x", 0.0f);
  cfg.drone_to_camera.y() = yaml_get(m, "drone_to_camera_y", 0.0f);
  cfg.drone_to_camera.z() = yaml_get(m, "drone_to_camera_z", 0.0f);

  cfg.servo_open_pwm = yaml_get(m, "servo_open_position", 1980.0f);
  cfg.servo_close_pwm = yaml_get(m, "servo_close_position", 1200.0f);
  cfg.prefer_large_target = yaml_get(m, "shot_big_target", true);

  // Default waypoint patterns (legacy hardcoded values)
  cfg.shot_zone.waypoints = {
    {0.33333f, 0.85f}, {0.0f, 0.85f}, {0.0f, 0.5f},  {0.33333f, 0.5f},
    {0.66666f, 0.5f},  {1.0f, 0.5f},  {1.0f, 0.85f}, {0.66666f, 0.85f},
    {0.66666f, 0.15f}, {1.0f, 0.15f}, {0.0f, 0.15f}, {0.33333f, 0.15f},
  };
  cfg.recon_zone.waypoints = {
    {0.0f, 1.0f}, {-0.5f, 0.0f}, {1.0f, 0.0f}, {1.5f, 1.0f}, {0.5f, 1.0f},
  };

  // Airdrop config
  YAML::Node a = ConfigLoader::load(airdrop_yaml);
  cfg.airdrop.pid = load_pid(a["pid"]);
  cfg.airdrop.limits = load_limits(a["limits"]);
  cfg.airdrop.radius = yaml_get(a, "radius", 0.08f);
  cfg.airdrop.accuracy = yaml_get(a, "accuracy", 0.75f);
  cfg.airdrop.shot_duration = yaml_get(a, "shot_duration", 0.6f);
  cfg.airdrop.shot_wait = yaml_get(a, "shot_wait", 0.8f);
  cfg.airdrop.tar_z = yaml_get(a, "tar_z", 1.0f);
  cfg.airdrop.shot_point_left = {
    yaml_get(a, "shot_target_x_l", -0.05f), yaml_get(a, "shot_target_y_l", 0.015f),
    yaml_get(a, "shot_target_z_l", 0.07f)};
  cfg.airdrop.shot_point_right = {
    yaml_get(a, "shot_target_x_r", 0.05f), yaml_get(a, "shot_target_y_r", 0.015f),
    yaml_get(a, "shot_target_z_r", 0.07f)};
  cfg.airdrop.tar_pixel_left = {yaml_get(a, "tar_x_l", 665.0f), yaml_get(a, "tar_y_l", 470.0f)};
  cfg.airdrop.tar_pixel_right = {yaml_get(a, "tar_x_r", 615.0f), yaml_get(a, "tar_y_r", 470.0f)};

  // Landing config
  YAML::Node l = ConfigLoader::load(landing_yaml);
  cfg.landing.pid = load_pid(l["pid"]);
  cfg.landing.limits = load_limits(l["limits"]);
  cfg.landing.scout_halt = yaml_get(l, "scout_halt", 2.5f);
  cfg.landing.scout_x = yaml_get(l, "scout_x", 0.0f);
  cfg.landing.scout_y = yaml_get(l, "scout_y", 0.0f);
  cfg.landing.accuracy = yaml_get(l, "accuracy", 0.00005f);
  cfg.landing.tar_z = yaml_get(l, "tar_z", 0.2f);
  cfg.landing.tar_pixel = {yaml_get(l, "tar_x", 640.0f), yaml_get(l, "tar_y", 400.0f)};

  return cfg;
}

}  // namespace drone::mission
