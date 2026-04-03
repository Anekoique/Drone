// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/mission/mission_types.hpp"

#include <Eigen/Core>

namespace drone::mission
{

/// Handles the init and takeoff phases of the mission.
class TakeoffHandler
{
public:
  TakeoffHandler(Subsystems & subs, const MissionConfig & config);

  /// Initialize coordinate frames, capture start position, set home.
  /// @return Next state (takeoff).
  FlyState initialize();

  /// Execute takeoff. Returns goto_shot when altitude reached.
  FlyState takeoff();

  /// Start position and yaw captured during initialize().
  Eigen::Vector4f start_position() const { return start_pos_; }
  /// Start yaw in radians.
  float start_yaw() const { return start_pos_.w(); }

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  Eigen::Vector4f start_pos_ = Eigen::Vector4f::Zero();
  bool init_done_ = false;
};

}  // namespace drone::mission
