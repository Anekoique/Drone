#pragma once

#include "drone/mission/mission_types.hpp"

#include <Eigen/Core>

namespace drone::mission
{

class TakeoffHandler
{
public:
  TakeoffHandler(Subsystems & subs, const MissionConfig & config);

  /// Initialize coordinate frames, capture start position, set home.
  FlyState initialize();

  /// Execute takeoff. Returns goto_shot when altitude reached.
  FlyState takeoff();

  Eigen::Vector4f start_position() const { return start_pos_; }
  float start_yaw() const { return start_pos_.w(); }

private:
  Subsystems & subs_;
  const MissionConfig & config_;
  Eigen::Vector4f start_pos_ = Eigen::Vector4f::Zero();
  bool init_done_ = false;
};

}  // namespace drone::mission
