// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

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

/// Top-level mission state machine states.
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
/// @param state Current fly state.
/// @return Integer code for ROS topic.
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

/// Bundle of references to all hardware subsystems, passed to handlers.
struct Subsystems
{
  Motors & motors;
  InertialNav & inav;
  control::PosControl & pos_control;
  Servo & servo;
  Gimbal & gimbal;
};

// --- Config Structs ---

/// Configuration for a rectangular zone (shot or recon area).
struct ZoneConfig
{
  float dx = 0;                            ///< Zone center X offset in compass frame (m)
  float dy = 0;                            ///< Zone center Y offset in compass frame (m)
  float length = 8.0f;                     ///< Zone length (m)
  float width = 5.0f;                      ///< Zone width (m)
  float altitude = 4.5f;                   ///< Cruise altitude (m)
  float altitude_surround = 3.0f;          ///< Search altitude (m)
  float altitude_low = 1.8f;               ///< Low-pass altitude (m)
  std::vector<Eigen::Vector2f> waypoints;  ///< Normalized 0-1 waypoints within zone
};

/// Airdrop (ball-drop) configuration.
struct AirdropConfig
{
  PID::Defaults pid;                                          ///< Visual servo PID gains
  control::Limits limits;                                     ///< Speed limits during airdrop
  float radius = 0.08f;                                       ///< Target container radius (m)
  float accuracy = 0.75f;                                     ///< Pixel accuracy for alignment
  float shot_duration = 0.6f;                                 ///< Servo open duration (s)
  float shot_wait = 0.8f;                                     ///< Wait after shot before next (s)
  float tar_z = 1.0f;                                         ///< Target altitude for airdrop (m)
  Eigen::Vector3f shot_point_left = {-0.05f, 0.015f, 0.07f};  ///< Left release offset
  Eigen::Vector3f shot_point_right = {0.05f, 0.015f, 0.07f};  ///< Right release offset
  Eigen::Vector2f tar_pixel_left = {665, 470};                ///< Left target pixel
  Eigen::Vector2f tar_pixel_right = {615, 470};               ///< Right target pixel
};

/// Landing (precision landing) configuration.
struct LandingConfig
{
  PID::Defaults pid;                       ///< Visual servo PID gains
  control::Limits limits;                  ///< Speed limits during landing
  float scout_halt = 2.5f;                 ///< Wait time at scout position (s)
  float scout_x = 0;                       ///< Scout X offset
  float scout_y = 0;                       ///< Scout Y offset
  float accuracy = 0.00005f;               ///< Pixel accuracy for alignment
  float tar_z = 0.2f;                      ///< Final approach altitude (m)
  Eigen::Vector2f tar_pixel = {640, 400};  ///< Target pixel for H alignment
  float descent_speed = -0.2f;             ///< Final descent speed (m/s)
  float descent_duration = 1.0f;           ///< Final descent duration (s)
  float surround_range = 3.0f;             ///< Search spiral range (m)
  float surround_step = 1.0f;              ///< Search spiral step size (m)
};

/// Complete mission configuration loaded from YAML files.
struct MissionConfig
{
  float heading_compass_deg = 180.0f;  ///< Compass heading in degrees
  float heading_compass_rad = 0;       ///< Compass heading in radians
  float heading_real_rad = 0;          ///< Real heading after yaw correction
  float default_yaw = 0;               ///< Default yaw for waypoints
  float bucket_height = 0.3f;          ///< Height of target containers (m)

  ZoneConfig shot_zone;   ///< Airdrop zone configuration
  ZoneConfig recon_zone;  ///< Reconnaissance zone configuration
  AirdropConfig airdrop;  ///< Airdrop parameters
  LandingConfig landing;  ///< Landing parameters

  float servo_open_pwm = 1980;      ///< Servo open PWM
  float servo_close_pwm = 1200;     ///< Servo close PWM
  bool prefer_large_target = true;  ///< If true, drop on largest target first

  Eigen::Vector3f drone_to_camera = Eigen::Vector3f::Zero();  ///< Camera offset from drone CG
  float takeoff_altitude = 2.0f;                              ///< Target takeoff altitude (m)

  /// Load mission config from three YAML files.
  /// @param mission_yaml Main mission config filename.
  /// @param airdrop_yaml Airdrop-specific config filename.
  /// @param landing_yaml Landing-specific config filename.
  /// @return Populated MissionConfig.
  static MissionConfig load(
    const std::string & mission_yaml, const std::string & airdrop_yaml,
    const std::string & landing_yaml);
};

}  // namespace drone::mission
