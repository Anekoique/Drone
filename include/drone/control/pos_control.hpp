// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/control/cascade_controller.hpp"
#include "drone/control/fuzzy_pid.hpp"
#include "drone/control/limits.hpp"
#include "drone/control/mavros_commander.hpp"
#include "drone/control/trajectory_controller.hpp"
#include "drone/control/velocity_controller.hpp"
#include "drone/drivers/inertial_nav.hpp"
#include "drone/math/pid.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <string>
#include <vector>

namespace drone::control
{

/// Holds YAML-loaded fuzzy config with owned arrays.
/// Lifetime: lives in PosControl, outlives FuzzyPID init calls.
struct FuzzyConfig
{
  std::vector<float> mf_params;
  std::vector<std::array<float, kQfDefault>> rules;

  /// Per-axis fuzzy controller parameters.
  struct ControllerParams
  {
    float max_error = 100.0f;
    float max_delta_error = 100.0f;
  };
  std::array<ControllerParams, 8> controllers;

  /// Build all 8 FuzzyPID::Params pointing into owned data.
  /// Pointers are valid while FuzzyConfig lives.
  std::array<FuzzyPID::Params, 8> to_params_array() const;
};

/// Load fuzzy PID configuration from a YAML node.
/// @param yaml YAML node containing fuzzy config keys.
/// @return Populated FuzzyConfig.
FuzzyConfig load_fuzzy_config(const YAML::Node & yaml);

/// Per-method state for go_to_position (replaces static locals).
struct GoToState
{
  bool first_call = true;
};

/// Per-method state for hold_position.
struct HoldState
{
  Eigen::Vector3f start_pos = Eigen::Vector3f::Zero();
  bool initialized = false;
};

/// Top-level position controller facade.
/// Composes VelocityController, CascadeController, TrajectoryController, MavrosCommander.
class PosControl
{
public:
  /// @param node ROS 2 node for publishers and clock.
  /// @param inav Inertial navigation reference for pose feedback.
  /// @param config_path YAML config filename for PID gains and limits.
  PosControl(rclcpp::Node & node, InertialNav & inav, const std::string & config_path);

  PosControl(const PosControl &) = delete;
  PosControl & operator=(const PosControl &) = delete;
  PosControl(PosControl &&) = delete;
  PosControl & operator=(PosControl &&) = delete;

  /// Go to position using velocity PID. Returns true when within accuracy.
  /// @param target Target position and yaw (x, y, z, yaw).
  /// @param accuracy Distance threshold in meters.
  bool go_to_position(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Hold position using cascade PID + setpoint_raw. Returns true when stable.
  /// @param target Target position and yaw (x, y, z, yaw).
  /// @param accuracy Distance threshold in meters.
  bool hold_position(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Trajectory to absolute position (PID-controlled, with target caching).
  /// @param target Target position and yaw (x, y, z, yaw).
  /// @param accuracy Distance threshold in meters.
  bool trajectory_to(const Eigen::Vector4f & target, float accuracy = 0.05f);

  /// Trajectory to relative offset from current position.
  /// @param offset Relative offset and yaw (dx, dy, dz, yaw).
  /// @param accuracy Distance threshold in meters.
  bool trajectory_relative(const Eigen::Vector4f & offset, float accuracy = 0.05f);

  /// Circular trajectory at given height and angular velocity.
  /// @param a Ellipse semi-axis X.
  /// @param b Ellipse semi-axis Y.
  /// @param height Altitude.
  /// @param angular_vel Angular velocity (rad/s).
  /// @param yaw Target yaw.
  bool circle(float a, float b, float height, float angular_vel, float yaw);

  /// S-curve via TrajectoryGenerator.
  /// @param speed_factor Speed scaling factor.
  /// @param q_goal Goal position [x, y, z].
  /// @param target_yaw Target yaw.
  /// @param max_speed Per-axis maximum speed.
  /// @param max_accel Per-axis maximum acceleration.
  bool trajectory_generator(
    double speed_factor, const std::array<double, 3> & q_goal, float target_yaw,
    const Eigen::Vector3f & max_speed = Eigen::Vector3f::Constant(100.0f),
    const Eigen::Vector3f & max_accel = Eigen::Vector3f::Constant(100.0f));

  /// Set control timestep.
  /// @param dt Time step in seconds.
  void set_dt(float dt);

  /// Reset all controllers and state.
  void reset();

  /// Reset trajectory state (call before new goal).
  void new_goal();

  /// Access sub-controllers for advanced use.
  VelocityController & velocity_controller() { return vel_ctrl_; }
  CascadeController & cascade_controller() { return cascade_ctrl_; }
  TrajectoryController & trajectory_controller() { return traj_ctrl_; }
  MavrosCommander & commander() { return commander_; }

  /// Override kinematic limits.
  void set_limits(const Limits & limits);

  /// Restore default kinematic limits.
  void reset_limits();

  /// @return Current default kinematic limits.
  Limits get_limits() const { return default_limits_; }

  /// Get current time from ROS clock (seconds).
  double get_time() const;

private:
  rclcpp::Node & node_;
  InertialNav & inav_;

  VelocityController vel_ctrl_;
  CascadeController cascade_ctrl_;
  TrajectoryController traj_ctrl_;
  MavrosCommander commander_;
  FuzzyPID fuzzy_pid_;
  FuzzyConfig fuzzy_config_;

  Limits default_limits_;
  float dt_ = 0.1f;

  GoToState goto_state_;
  HoldState hold_state_;
};

}  // namespace drone::control
