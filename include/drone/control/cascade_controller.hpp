#pragma once

#include "drone/control/limits.hpp"
#include "drone/control/yaw_utils.hpp"
#include "drone/math/pid.hpp"

#include <Eigen/Core>

namespace drone::control
{

/// Cascade position -> velocity -> acceleration controller.
/// Ports legacy input_pos_vel_1_xyz_yaw / input_pos_vel_xyz_yaw.
class CascadeController
{
public:
  struct Config
  {
    PID::Defaults pid_px;
    PID::Defaults pid_py;
    PID::Defaults pid_pz;
    PID::Defaults pid_vx;
    PID::Defaults pid_vy;
    PID::Defaults pid_vz;
    PID::Defaults pid_yaw;
    Limits limits;
    float dt_outer = 0.1f;  // position -> velocity loop timestep
    float dt_inner = 1.0f;  // velocity -> acceleration loop timestep
  };

  explicit CascadeController(const Config & config);
  CascadeController() = default;

  /// Full cascade: position error -> velocity -> acceleration command.
  /// Uses dt_outer for position PIDs and dt_inner for velocity PIDs.
  Eigen::Vector4f compute(
    const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target,
    const Eigen::Vector4f & vel_current, float yaw_current, float yaw_target);

  void set_limits(const Limits & limits);
  void reset_pid();
  void set_pid_config(const Config & config);

private:
  PID pid_px_, pid_py_, pid_pz_;
  PID pid_vx_, pid_vy_, pid_vz_;
  PID pid_yaw_;
  Limits limits_;
  float dt_outer_ = 0.1f;
  float dt_inner_ = 1.0f;
};

}  // namespace drone::control
