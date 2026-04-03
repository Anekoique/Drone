#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/utils/timer.hpp"

#include <Eigen/Core>

namespace drone::mission
{

class LandingHandler
{
public:
  LandingHandler(Subsystems & subs, const MissionConfig & config);

  /// Doland: RTL -> wait -> visual approach -> descent -> LAND.
  FlyState execute(float heading_rad, float start_yaw);

  /// LandToStart: fly to (0,0,2) -> wait 19s -> LAND.
  FlyState land_to_start();

  void reset();
  void reset_land_to_start();

private:
  enum class DolandState
  {
    rtl,
    wait,
    visual_approach,
    descent,
    land
  };

  enum class LandToStartState
  {
    init,
    wait,
    land
  };

  Subsystems & subs_;
  const MissionConfig & config_;

  DolandState doland_state_ = DolandState::rtl;
  LandToStartState lts_state_ = LandToStartState::init;
  Timer state_timer_;

  // Visual approach state
  int surround_land_ = -3;
  Timer target_loss_timer_;
  bool approach_initialized_ = false;
};

}  // namespace drone::mission
