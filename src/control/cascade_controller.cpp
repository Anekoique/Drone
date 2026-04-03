// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file cascade_controller.cpp
/// @brief Two-loop cascade controller: outer loop (position -> velocity target,
///        P-only) feeds an inner loop (velocity -> acceleration, full PID).
///        Used for precision position hold with acceleration output.

#include "drone/control/cascade_controller.hpp"

#include <cassert>
#include <cmath>

namespace drone::control
{

CascadeController::CascadeController(const Config & config)
: pid_px_(config.pid_px),
  pid_py_(config.pid_py),
  pid_pz_(config.pid_pz),
  pid_vx_(config.pid_vx),
  pid_vy_(config.pid_vy),
  pid_vz_(config.pid_vz),
  pid_yaw_(config.pid_yaw),
  limits_(config.limits),
  dt_outer_(config.dt_outer),
  dt_inner_(config.dt_inner)
{
  // Outer-loop position PIDs are called with dt=0 (P-only mode).
  // Non-zero I or D gains would cause division-by-zero or NaN propagation.
  assert(
    config.pid_px.i == 0.0f && config.pid_px.d == 0.0f &&
    "Outer-loop cascade PIDs must be P-only (i=0, d=0)");
  assert(
    config.pid_py.i == 0.0f && config.pid_py.d == 0.0f &&
    "Outer-loop cascade PIDs must be P-only (i=0, d=0)");
  assert(
    config.pid_pz.i == 0.0f && config.pid_pz.d == 0.0f &&
    "Outer-loop cascade PIDs must be P-only (i=0, d=0)");
}

/// Run both cascade loops: outer P-only position loop generates velocity targets,
/// inner full-PID velocity loop generates acceleration commands.
/// Yaw uses a single-loop PID with shortest-path angular wrapping.
Eigen::Vector4f CascadeController::compute(
  const Eigen::Vector3f & pos_current, const Eigen::Vector3f & pos_target,
  const Eigen::Vector4f & vel_current, float yaw_current, float yaw_target)
{
  Eigen::Vector4f accel;

  // Outer loop: position -> velocity target (using dt_outer = 0 means P-only for position)
  float vel_target_x =
    pid_px_.update_all(pos_current.x(), pos_target.x(), 0, limits_.speed_max_xy, vel_current.x());
  float vel_target_y =
    pid_py_.update_all(pos_current.y(), pos_target.y(), 0, limits_.speed_max_xy, vel_current.y());
  float vel_target_z =
    pid_pz_.update_all(pos_current.z(), pos_target.z(), 0, limits_.speed_max_z, vel_current.z());

  // Inner loop: velocity -> acceleration (using dt_inner matching legacy dt_pid_p_v)
  accel.x() = pid_vx_.update_all(vel_current.x(), vel_target_x, dt_inner_, limits_.accel_max_xy);
  accel.y() = pid_vy_.update_all(vel_current.y(), vel_target_y, dt_inner_, limits_.accel_max_xy);
  accel.z() = pid_vz_.update_all(vel_current.z(), vel_target_z, dt_inner_, limits_.accel_max_z);

  // Yaw: single-loop PID
  float yaw_adjusted = yaw_current + yaw_error(yaw_current, yaw_target);
  accel.w() = pid_yaw_.update_all(
    yaw_current, yaw_adjusted, dt_outer_, limits_.speed_max_yaw, vel_current.w());

  return accel;
}

void CascadeController::set_limits(const Limits & limits)
{
  limits_ = limits;
}

void CascadeController::reset_pid()
{
  pid_px_.reset_all();
  pid_py_.reset_all();
  pid_pz_.reset_all();
  pid_vx_.reset_all();
  pid_vy_.reset_all();
  pid_vz_.reset_all();
  pid_yaw_.reset_all();
}

void CascadeController::set_pid_config(const Config & config)
{
  pid_px_.set_pid(config.pid_px);
  pid_py_.set_pid(config.pid_py);
  pid_pz_.set_pid(config.pid_pz);
  pid_vx_.set_pid(config.pid_vx);
  pid_vy_.set_pid(config.pid_vy);
  pid_vz_.set_pid(config.pid_vz);
  pid_yaw_.set_pid(config.pid_yaw);
}

}  // namespace drone::control
