// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file pos_control.cpp
/// @brief Top-level position controller that dispatches to velocity, cascade,
///        trajectory, and fuzzy-PID sub-controllers depending on the flight mode.

#include "drone/control/pos_control.hpp"

#include "drone/utils/readyaml.hpp"

#include <cmath>
#include <stdexcept>

namespace drone::control
{

// --- FuzzyConfig ---

/// Build a FuzzyPID::Params array from the YAML-loaded fuzzy config.
/// Each of the 8 controllers shares the same membership functions and rule base
/// but has its own error/delta-error scaling.
std::array<FuzzyPID::Params, 8> FuzzyConfig::to_params_array() const
{
  std::array<FuzzyPID::Params, 8> params{};
  for (int i = 0; i < 8; ++i) {
    params[i].mf_type = 4;  // trapezoidal
    params[i].fo_type = 1;
    params[i].df_type = 0;
    params[i].mf_params = mf_params.data();
    params[i].rule_base = reinterpret_cast<const float (*)[kQfDefault]>(rules.data());
    params[i].max_error = controllers[i].max_error;
    params[i].max_delta_error = controllers[i].max_delta_error;
    params[i].control_id_count = 8;
  }
  return params;
}

/// Parse fuzzy controller configuration from YAML.
/// Expects mf_params (membership function breakpoints), a 21-row rule_base,
/// and per-controller max_error / max_delta_error entries.
FuzzyConfig load_fuzzy_config(const YAML::Node & yaml)
{
  FuzzyConfig config;

  if (yaml["mf_params"]) {
    for (const auto & v : yaml["mf_params"]) {
      config.mf_params.push_back(v.as<float>());
    }
  }

  if (yaml["rule_base"]) {
    for (const auto & row : yaml["rule_base"]) {
      std::array<float, kQfDefault> r{};
      for (int i = 0; i < kQfDefault && i < static_cast<int>(row.size()); ++i) {
        r[i] = row[i].as<float>();
      }
      config.rules.push_back(r);
    }
  }

  if (yaml["controllers"]) {
    int idx = 0;
    for (const auto & c : yaml["controllers"]) {
      if (idx >= 8) break;
      config.controllers[idx].max_error = c["max_error"].as<float>();
      config.controllers[idx].max_delta_error = c["max_delta_error"].as<float>();
      ++idx;
    }
  }

  if (config.rules.size() != 21) {
    throw std::runtime_error(
      "fuzzy rule_base must have 21 rows, got " + std::to_string(config.rules.size()));
  }

  return config;
}

// --- PosControl ---

/// Load all sub-controller configurations from the YAML config file and
/// initialize velocity, cascade, trajectory, and fuzzy-PID controllers.
PosControl::PosControl(rclcpp::Node & node, InertialNav & inav, const std::string & config_path)
: node_(node), inav_(inav), commander_(node, "/mavros/")
{
  YAML::Node config = ConfigLoader::load(config_path);

  // Load limits
  default_limits_ = load_limits(config["limits"]);

  // Load simple PID configs
  VelocityController::Config vel_config;
  vel_config.pid_x = PID::readPIDParameters(config_path, "pos_x");
  vel_config.pid_y = PID::readPIDParameters(config_path, "pos_y");
  vel_config.pid_z = PID::readPIDParameters(config_path, "pos_z");
  vel_config.pid_yaw = PID::readPIDParameters(config_path, "pos_yaw");
  vel_config.limits = default_limits_;
  vel_ctrl_ = VelocityController(vel_config);

  // Load cascade PID configs
  CascadeController::Config cas_config;
  cas_config.pid_px = PID::readPIDParameters(config_path, "pos_px");
  cas_config.pid_py = PID::readPIDParameters(config_path, "pos_py");
  cas_config.pid_pz = PID::readPIDParameters(config_path, "pos_pz");
  cas_config.pid_vx = PID::readPIDParameters(config_path, "pos_vx");
  cas_config.pid_vy = PID::readPIDParameters(config_path, "pos_vy");
  cas_config.pid_vz = PID::readPIDParameters(config_path, "pos_vz");
  cas_config.pid_yaw = PID::readPIDParameters(config_path, "pos_yaw");
  cas_config.limits = default_limits_;
  if (config["cascade"]) {
    if (config["cascade"]["dt_outer"])
      cas_config.dt_outer = config["cascade"]["dt_outer"].as<float>();
    if (config["cascade"]["dt_inner"])
      cas_config.dt_inner = config["cascade"]["dt_inner"].as<float>();
  }
  cascade_ctrl_ = CascadeController(cas_config);

  // Load trajectory controller
  traj_ctrl_ = TrajectoryController(default_limits_);

  // Load fuzzy config
  if (config["fuzzy"]) {
    fuzzy_config_ = load_fuzzy_config(config["fuzzy"]);
    auto params = fuzzy_config_.to_params_array();
    fuzzy_pid_.init(params.data());
  }
}

/// Fly toward an absolute position target using single-loop velocity PID.
/// On the first call a position setpoint is published; subsequent calls send
/// velocity commands. Returns true once all axes are within the accuracy threshold.
bool PosControl::go_to_position(const Eigen::Vector4f & target, float accuracy)
{
  Eigen::Vector3f pos = inav_.position();
  float yaw_current = inav_.yaw();

  // First call: publish position setpoint
  if (goto_state_.first_call) {
    commander_.send_position(target);
    goto_state_.first_call = false;
  }

  // Check arrival
  if (
    std::abs(pos.x() - target.x()) <= accuracy && std::abs(pos.y() - target.y()) <= accuracy &&
    std::abs(pos.z() - target.z()) <= accuracy) {
    goto_state_ = GoToState{};
    return true;
  }

  // PID velocity control
  Eigen::Vector4f vel =
    vel_ctrl_.compute(pos, target.head<3>(), yaw_current, target.w(), dt_, inav_.velocity());
  commander_.send_velocity(vel);
  return false;
}

/// Hold a position using the cascade (position -> velocity -> acceleration) controller.
/// Resets PIDs on the first call, then continuously sends acceleration commands.
bool PosControl::hold_position(const Eigen::Vector4f & target, float accuracy)
{
  Eigen::Vector3f pos = inav_.position();
  float yaw_current = inav_.yaw();

  if (!hold_state_.initialized) {
    hold_state_.start_pos = pos;
    hold_state_.initialized = true;
    vel_ctrl_.reset_pid();
    cascade_ctrl_.reset_pid();
  }

  // Use cascade controller: position -> velocity -> acceleration
  Eigen::Vector4f accel =
    cascade_ctrl_.compute(pos, target.head<3>(), inav_.velocity(), yaw_current, target.w());
  commander_.send_acceleration(accel);

  if (
    std::abs(pos.x() - target.x()) <= accuracy && std::abs(pos.y() - target.y()) <= accuracy &&
    std::abs(pos.z() - target.z()) <= accuracy) {
    hold_state_ = HoldState{};
    return true;
  }
  return false;
}

/// Follow a trajectory to an absolute target using fuzzy-PID velocity control.
/// The trajectory controller caches the target and the fuzzy-PID adapts gains online.
bool PosControl::trajectory_to(const Eigen::Vector4f & target, float accuracy)
{
  Eigen::Vector3f pos = inav_.position();
  float yaw_current = inav_.yaw();

  auto [waypoint, done] = traj_ctrl_.setpoint_world(pos, target, accuracy);

  if (!done) {
    Eigen::Vector4f vel = vel_ctrl_.compute_fuzzy(
      pos, waypoint.head<3>(), yaw_current, waypoint.w(), dt_, inav_.velocity(), fuzzy_pid_);
    commander_.send_velocity(vel);
  } else {
    vel_ctrl_.reset_pid();
  }

  return done;
}

/// Follow a trajectory to a position offset relative to the current pose.
/// Internally converted to absolute coordinates on the first call.
bool PosControl::trajectory_relative(const Eigen::Vector4f & offset, float accuracy)
{
  Eigen::Vector3f pos = inav_.position();
  float yaw_current = inav_.yaw();

  auto [waypoint, done] = traj_ctrl_.setpoint_relative(pos, offset, accuracy);

  if (!done) {
    Eigen::Vector4f vel = vel_ctrl_.compute_fuzzy(
      pos, waypoint.head<3>(), yaw_current, waypoint.w(), dt_, inav_.velocity(), fuzzy_pid_);
    commander_.send_velocity(vel);
  } else {
    vel_ctrl_.reset_pid();
  }

  return done;
}

/// Execute an elliptical orbit with semi-axes (a, b) using position + velocity
/// feedforward via setpoint_raw. The velocity feedforward is the analytic
/// derivative of the parametric ellipse.
bool PosControl::circle(float a, float b, float height, float angular_vel, float yaw)
{
  auto [waypoint, done] = traj_ctrl_.circle(a, b, height, angular_vel, yaw);

  // Circle uses setpoint_raw with position + velocity feedforward
  Eigen::Vector4f circle_vel(
    -a * std::sin(waypoint.w()) * angular_vel, b * std::cos(waypoint.w()) * angular_vel, 0, 0);
  commander_.publish_setpoint_raw(waypoint, circle_vel);

  return done;
}

/// Fly a smooth trajectory generated by TrajectoryGenerator (time-optimal
/// polynomial planner). Uses fuzzy-PID velocity control to track the planned waypoints.
bool PosControl::trajectory_generator(
  double speed_factor, const std::array<double, 3> & q_goal, float target_yaw,
  const Eigen::Vector3f & max_speed, const Eigen::Vector3f & max_accel)
{
  Eigen::Vector3f pos = inav_.position();
  float yaw_current = inav_.yaw();

  auto [waypoint, done] =
    traj_ctrl_.generator_world(pos, speed_factor, q_goal, target_yaw, max_speed, max_accel);

  if (!done) {
    // Legacy uses relative coordinates for PID: pos - start, waypoint - start
    Eigen::Vector4f vel = vel_ctrl_.compute_fuzzy(
      pos, waypoint.head<3>(), yaw_current, waypoint.w(), dt_, inav_.velocity(), fuzzy_pid_);
    commander_.send_velocity(vel);
  } else {
    vel_ctrl_.reset_pid();
  }

  return done;
}

void PosControl::set_dt(float dt)
{
  dt_ = dt;
}

void PosControl::reset()
{
  vel_ctrl_.reset_pid();
  cascade_ctrl_.reset_pid();
  traj_ctrl_.reset_all();
  goto_state_ = GoToState{};
  hold_state_ = HoldState{};
}

void PosControl::new_goal()
{
  traj_ctrl_.reset_all();
  vel_ctrl_.reset_pid();
  cascade_ctrl_.reset_pid();
  goto_state_ = GoToState{};
  hold_state_ = HoldState{};
}

void PosControl::set_limits(const Limits & limits)
{
  vel_ctrl_.set_limits(limits);
  cascade_ctrl_.set_limits(limits);
}

void PosControl::reset_limits()
{
  set_limits(default_limits_);
}

double PosControl::get_time() const
{
  return node_.get_clock()->now().seconds();
}

}  // namespace drone::control
