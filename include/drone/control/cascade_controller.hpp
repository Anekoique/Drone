// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

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
  /// Configuration with outer (position) and inner (velocity) PID gains.
  struct Config
  {
    PID::Defaults pid_px;   ///< Position X PID gains
    PID::Defaults pid_py;   ///< Position Y PID gains
    PID::Defaults pid_pz;   ///< Position Z PID gains
    PID::Defaults pid_vx;   ///< Velocity X PID gains
    PID::Defaults pid_vy;   ///< Velocity Y PID gains
    PID::Defaults pid_vz;   ///< Velocity Z PID gains
    PID::Defaults pid_yaw;  ///< Yaw PID gains
    Limits limits;
    float dt_outer = 0.1f;  ///< Position -> velocity loop timestep
    float dt_inner = 1.0f;  ///< Velocity -> acceleration loop timestep
  };

  explicit CascadeController(const Config & config);
  CascadeController() = default;

  /// Full cascade: position error -> velocity -> acceleration command.
  /// @param pos_current Current position (x, y, z).
  /// @param pos_target Target position (x, y, z).
  /// @param vel_current Current velocity (vx, vy, vz, yaw_rate).
  /// @param yaw_current Current yaw (radians).
  /// @param yaw_target Target yaw (radians).
  /// @return Acceleration command (ax, ay, az, yaw_accel).
  Eigen::Vector4f compute(
    const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target,
    const Eigen::Vector4f & vel_current, float yaw_current, float yaw_target);

  /// Update kinematic limits.
  void set_limits(const Limits & limits);

  /// Reset all PID integrators.
  void reset_pid();

  /// Reconfigure all PID gains and limits.
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
