// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/mission/target_tracker.hpp"
#include "drone/utils/timer.hpp"

#include <Eigen/Core>

namespace drone::mission
{

/// Handles precision landing using visual servo on the H target.
class LandingHandler
{
public:
  LandingHandler(Subsystems & subs, const MissionConfig & config);

  /// Set the target tracker (must be called before execute).
  void set_tracker(TargetTracker * tracker) { tracker_ = tracker; }

  /// Precision landing: RTL -> wait -> visual approach -> descent -> LAND.
  /// @param heading_rad Heading for coordinate transforms.
  /// @param start_yaw Startup yaw.
  /// @return land_to_start or finished when complete.
  FlyState execute(float heading_rad, float start_yaw);

  /// Simple landing: fly to (0, 0, 2) -> wait 19s -> LAND.
  /// @return finished when landing command sent.
  FlyState land_to_start();

  /// Reset precision landing state.
  void reset();
  /// Reset land-to-start state.
  void reset_land_to_start();

private:
  /// Precision landing sub-states.
  enum class DolandState
  {
    rtl,
    wait,
    visual_approach,
    descent,
    land
  };

  /// Land-to-start sub-states.
  enum class LandToStartState
  {
    init,
    wait,
    land
  };

  Subsystems & subs_;
  const MissionConfig & config_;
  TargetTracker * tracker_ = nullptr;

  DolandState doland_state_ = DolandState::rtl;
  LandToStartState lts_state_ = LandToStartState::init;
  Timer state_timer_;

  // Visual approach state
  int surround_land_ = -3;  ///< Search spiral step counter
  Timer target_loss_timer_;
  bool approach_initialized_ = false;
};

}  // namespace drone::mission
