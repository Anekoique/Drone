#pragma once

#include "drone/control/limits.hpp"
#include "drone/control/pos_control.hpp"
#include "drone/drivers/gimbal.hpp"
#include "drone/drivers/motors.hpp"
#include "drone/drivers/servo.hpp"
#include "drone/math/pid.hpp"

#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace drone::mission
{

// --- FlyState ---

enum class FlyState
{
  init,
  takeoff,
  goto_shot,
  airdrop,
  goto_recon,
  recon_patrol,
  landing,
  land_to_start,
  finished,
};

/// Legacy-compatible integer encoding for state publisher.
inline int fly_state_to_int(FlyState state)
{
  switch (state) {
    case FlyState::init:
      return 1;
    case FlyState::takeoff:
      return 1;
    case FlyState::goto_shot:
      return 1;
    case FlyState::airdrop:
      return 0;
    case FlyState::goto_recon:
      return 1;
    case FlyState::recon_patrol:
      return 3;
    case FlyState::landing:
      return 4;
    case FlyState::land_to_start:
      return 4;
    case FlyState::finished:
      return 2;
    default:
      return 1;
  }
}

// --- Subsystems ---

struct Subsystems
{
  Motors & motors;
  InertialNav & inav;
  control::PosControl & pos_control;
  Servo & servo;
  Gimbal & gimbal;
};

// --- Config Structs ---

struct ZoneConfig
{
  float dx = 0;
  float dy = 0;
  float length = 8.0f;
  float width = 5.0f;
  float altitude = 4.5f;
  float altitude_surround = 3.0f;
  float altitude_low = 1.8f;
  std::vector<Eigen::Vector2f> waypoints;
};

struct AirdropConfig
{
  PID::Defaults pid;
  control::Limits limits;
  float radius = 0.08f;
  float accuracy = 0.75f;
  float shot_duration = 0.6f;
  float shot_wait = 0.8f;
  float tar_z = 1.0f;
  Eigen::Vector3f shot_point_left = {-0.05f, 0.015f, 0.07f};
  Eigen::Vector3f shot_point_right = {0.05f, 0.015f, 0.07f};
  Eigen::Vector2f tar_pixel_left = {665, 470};
  Eigen::Vector2f tar_pixel_right = {615, 470};
};

struct LandingConfig
{
  PID::Defaults pid;
  control::Limits limits;
  float scout_halt = 2.5f;
  float scout_x = 0;
  float scout_y = 0;
  float accuracy = 0.00005f;
  float tar_z = 0.2f;
  Eigen::Vector2f tar_pixel = {640, 400};
  float descent_speed = -0.2f;
  float descent_duration = 1.0f;
  float surround_range = 3.0f;
  float surround_step = 1.0f;
};

struct MissionConfig
{
  float heading_compass_deg = 180.0f;
  float heading_compass_rad = 0;
  float heading_real_rad = 0;
  float default_yaw = 0;
  float bucket_height = 0.3f;

  ZoneConfig shot_zone;
  ZoneConfig recon_zone;
  AirdropConfig airdrop;
  LandingConfig landing;

  float servo_open_pwm = 1980;
  float servo_close_pwm = 1200;
  bool prefer_large_target = true;

  Eigen::Vector3f drone_to_camera = Eigen::Vector3f::Zero();
  float takeoff_altitude = 2.0f;

  static MissionConfig load(
    const std::string & mission_yaml, const std::string & airdrop_yaml,
    const std::string & landing_yaml);
};

}  // namespace drone::mission
