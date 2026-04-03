// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file takeoff_handler.cpp
/// @brief Takeoff initialization and execution: captures start position/yaw,
///        sets home, configures gimbal, and delegates to the Motors takeoff FSM.

#include "drone/mission/takeoff_handler.hpp"

namespace drone::mission
{

TakeoffHandler::TakeoffHandler(Subsystems & subs, const MissionConfig & config)
: subs_(subs), config_(config)
{
}

FlyState TakeoffHandler::initialize()
{
  if (init_done_) return FlyState::takeoff;

  // Capture startup position
  Eigen::Vector3f pos = subs_.inav.position();
  float yaw = subs_.inav.yaw();
  start_pos_ = Eigen::Vector4f(pos.x(), pos.y(), pos.z(), yaw);

  // Set home position
  subs_.motors.set_home_position(yaw);

  // Initialize gimbal (pitch down for camera)
  subs_.gimbal.set_gimbal(-90.0f, 0.0f, 0.0f);

  // Switch to GUIDED mode
  subs_.motors.switch_mode("GUIDED");

  init_done_ = true;
  return FlyState::takeoff;
}

FlyState TakeoffHandler::takeoff()
{
  bool done =
    subs_.motors.takeoff(subs_.inav.position().z(), config_.takeoff_altitude, start_pos_.w());

  return done ? FlyState::goto_shot : FlyState::takeoff;
}

}  // namespace drone::mission
