// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/control/limits.hpp"
#include "drone/trajectory/trajectory_generator.hpp"

#include <Eigen/Core>

#include <memory>
#include <utility>

namespace drone::control
{

/// Per-method state for setpoint caching (replaces static locals from legacy).
struct SetpointState
{
  Eigen::Vector4f cached_target = Eigen::Vector4f::Zero();
  bool initialized = false;
};

/// Per-method state for circular trajectory.
struct CircleState
{
  float angle = 0.0f;
  bool active = false;
};

/// Per-method state for trajectory generator.
struct GeneratorState
{
  Eigen::Vector3f start_pos = Eigen::Vector3f::Zero();
  Eigen::Vector4f cached_target = Eigen::Vector4f::Zero();
  bool initialized = false;
  uint64_t count = 0;  ///< Iteration counter for the generator
};

/// Trajectory generation: returns waypoints for PosControl to feed to VelocityController.
/// No VelocityController dependency -- pure trajectory computation.
class TrajectoryController
{
public:
  explicit TrajectoryController(const Limits & limits);
  TrajectoryController() = default;

  /// S-curve trajectory to an absolute world position.
  /// On first call (after reset), caches target and triggers PID reset in caller.
  /// If target changes between calls, reinitializes automatically.
  /// @param pos_current Current position (x, y, z).
  /// @param pos_target Target position and yaw (x, y, z, yaw).
  /// @param accuracy Distance threshold for completion.
  /// @return {waypoint (x, y, z, yaw), done}.
  std::pair<Eigen::Vector4f, bool> setpoint_world(
    const Eigen::Vector3f & pos_current, const Eigen::Vector4f & pos_target,
    float accuracy = 0.05f);

  /// Relative-frame trajectory: target is offset from pos_current on first call.
  /// @param pos_current Current position (x, y, z).
  /// @param pos_offset Relative offset and yaw (dx, dy, dz, yaw).
  /// @param accuracy Distance threshold for completion.
  /// @return {waypoint (x, y, z, yaw), done}.
  std::pair<Eigen::Vector4f, bool> setpoint_relative(
    const Eigen::Vector3f & pos_current, const Eigen::Vector4f & pos_offset,
    float accuracy = 0.05f);

  /// Circular trajectory: returns position on a circle at given height.
  /// @param a Ellipse semi-axis X.
  /// @param b Ellipse semi-axis Y.
  /// @param height Altitude.
  /// @param angular_vel Angular velocity (rad/s).
  /// @param yaw Target yaw.
  /// @return {position (x, y, z, yaw), done}.
  std::pair<Eigen::Vector4f, bool> circle(
    float a, float b, float height, float angular_vel, float yaw);

  /// S-curve via TrajectoryGenerator: returns waypoint for smooth motion.
  /// @param pos_current Current position (x, y, z).
  /// @param speed_factor Speed scaling factor.
  /// @param q_goal Goal position array [x, y, z].
  /// @param target_yaw Target yaw.
  /// @param max_speed Per-axis max speed.
  /// @param max_accel Per-axis max acceleration.
  /// @return {waypoint (x, y, z, yaw), done}.
  std::pair<Eigen::Vector4f, bool> generator_world(
    const Eigen::Vector3f & pos_current, double speed_factor, const std::array<double, 3> & q_goal,
    float target_yaw, const Eigen::Vector3f & max_speed, const Eigen::Vector3f & max_accel);

  /// Reset setpoint_world state.
  void reset_setpoint();
  /// Reset setpoint_relative state.
  void reset_relative();
  /// Reset circle state.
  void reset_circle();
  /// Reset generator_world state.
  void reset_generator();
  /// Reset all trajectory states.
  void reset_all();

private:
  Limits limits_;
  std::unique_ptr<TrajectoryGenerator> generator_;

  SetpointState setpoint_state_;
  SetpointState relative_state_;
  CircleState circle_state_;
  GeneratorState generator_state_;
};

}  // namespace drone::control
