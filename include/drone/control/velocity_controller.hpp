// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/control/fuzzy_pid.hpp"
#include "drone/control/limits.hpp"
#include "drone/control/yaw_utils.hpp"
#include "drone/math/pid.hpp"

#include <Eigen/Core>

namespace drone::control
{

/// Single-loop position -> velocity controller (simple PID path).
/// Ports legacy input_pos_xyz / input_pos_xyz_yaw.
class VelocityController
{
public:
  /// PID gains and limits configuration.
  struct Config
  {
    PID::Defaults pid_x;
    PID::Defaults pid_y;
    PID::Defaults pid_z;
    PID::Defaults pid_yaw;
    Limits limits;
  };

  explicit VelocityController(const Config & config);
  VelocityController() = default;

  /// Compute velocity command from position error.
  /// @param pos_current Current position (x, y, z).
  /// @param pos_target Target position (x, y, z).
  /// @param yaw_current Current yaw (radians).
  /// @param yaw_target Target yaw (radians).
  /// @param dt Time step in seconds.
  /// @param velocity_feedback Current velocity from InertialNav (x, y, z, yaw_rate).
  /// @return Velocity command (vx, vy, vz, yaw_rate).
  Eigen::Vector4f compute(
    const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target, float yaw_current,
    float yaw_target, float dt, const Eigen::Vector4f & velocity_feedback);

  /// Compute with fuzzy gain adaptation.
  /// @param fuzzy FuzzyPID instance for online gain adjustment.
  /// @return Velocity command (vx, vy, vz, yaw_rate).
  Eigen::Vector4f compute_fuzzy(
    const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target, float yaw_current,
    float yaw_target, float dt, const Eigen::Vector4f & velocity_feedback, FuzzyPID & fuzzy);

  /// Update kinematic limits.
  void set_limits(const Limits & limits);

  /// Reset all PID integrators and state.
  void reset_pid();

  /// Reconfigure all PID gains and limits.
  void set_pid_config(const Config & config);

  PID & pid_x() { return pid_x_; }
  PID & pid_y() { return pid_y_; }
  PID & pid_z() { return pid_z_; }
  PID & pid_yaw() { return pid_yaw_; }

private:
  PID pid_x_;
  PID pid_y_;
  PID pid_z_;
  PID pid_yaw_;
  Limits limits_;
};

}  // namespace drone::control
