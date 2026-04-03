// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file velocity_controller.cpp
/// @brief Single-loop velocity controller with per-axis PID and yaw wrapping.
///        Also provides a fuzzy-PID variant that adapts gains before each compute step.

#include "drone/control/velocity_controller.hpp"

#include <cmath>

namespace drone::control
{

VelocityController::VelocityController(const Config & config)
: pid_x_(config.pid_x),
  pid_y_(config.pid_y),
  pid_z_(config.pid_z),
  pid_yaw_(config.pid_yaw),
  limits_(config.limits)
{
}

Eigen::Vector4f VelocityController::compute(
  const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target, float yaw_current,
  float yaw_target, float dt, const Eigen::Vector4f & velocity_feedback)
{
  Eigen::Vector4f vel;
  vel.x() = pid_x_.update_all(
    pos_current.x(), pos_target.x(), dt, limits_.speed_max_xy, velocity_feedback.x());
  vel.y() = pid_y_.update_all(
    pos_current.y(), pos_target.y(), dt, limits_.speed_max_xy, velocity_feedback.y());
  vel.z() = pid_z_.update_all(
    pos_current.z(), pos_target.z(), dt, limits_.speed_max_z, velocity_feedback.z());

  // Yaw wrapping: adjust target so PID sees the shortest angular path
  float yaw_adjusted = yaw_current + yaw_error(yaw_current, yaw_target);
  vel.w() = pid_yaw_.update_all(
    yaw_current, yaw_adjusted, dt, limits_.speed_max_yaw, velocity_feedback.w());

  return vel;
}

/// Fuzzy-augmented velocity control: for each axis, extract current PID gains,
/// let the fuzzy controller adjust them based on error/delta-error, apply the
/// updated gains, and then run the standard compute path.
Eigen::Vector4f VelocityController::compute_fuzzy(
  const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target, float yaw_current,
  float yaw_target, float dt, const Eigen::Vector4f & velocity_feedback, FuzzyPID & fuzzy)
{
  constexpr float delta_k = 2.0f;
  float kp = 0, ki = 0, kd = 0;

  pid_x_.get_pid(kp, ki, kd);
  fuzzy.fuzzy_pid_control(pos_current.x(), pos_target.x(), 0, kp, ki, kd, delta_k);
  pid_x_.set_gains(kp, ki, kd);

  pid_y_.get_pid(kp, ki, kd);
  fuzzy.fuzzy_pid_control(pos_current.y(), pos_target.y(), 1, kp, ki, kd, delta_k);
  pid_y_.set_gains(kp, ki, kd);

  pid_z_.get_pid(kp, ki, kd);
  fuzzy.fuzzy_pid_control(pos_current.z(), pos_target.z(), 2, kp, ki, kd, delta_k);
  pid_z_.set_gains(kp, ki, kd);

  pid_yaw_.get_pid(kp, ki, kd);
  fuzzy.fuzzy_pid_control(yaw_current, yaw_target, 3, kp, ki, kd, delta_k);
  pid_yaw_.set_gains(kp, ki, kd);

  return compute(pos_current, pos_target, yaw_current, yaw_target, dt, velocity_feedback);
}

void VelocityController::set_limits(const Limits & limits)
{
  limits_ = limits;
}

void VelocityController::reset_pid()
{
  pid_x_.reset_all();
  pid_y_.reset_all();
  pid_z_.reset_all();
  pid_yaw_.reset_all();
}

void VelocityController::set_pid_config(const Config & config)
{
  pid_x_.set_pid(config.pid_x);
  pid_y_.set_pid(config.pid_y);
  pid_z_.set_pid(config.pid_z);
  pid_yaw_.set_pid(config.pid_yaw);
}

}  // namespace drone::control
