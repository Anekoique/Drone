// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Core>

#include <array>

namespace drone
{

/// Robot state snapshot for trajectory generation.
struct RobotState
{
  std::array<double, 3> q_d{};  ///< Current desired joint/position values
};

/// Time-synchronized S-curve trajectory generator for 3 axes.
/// Computes smooth position deltas given kinematic constraints.
class TrajectoryGenerator
{
public:
  using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;

  TrajectoryGenerator() = default;

  /// @param speed_factor Scaling factor for maximum velocity.
  /// @param q_goal Goal position [x, y, z].
  TrajectoryGenerator(double speed_factor, std::array<double, 3> q_goal);

  /// Set per-axis maximum velocity.
  /// @param dq_max Array of [vx_max, vy_max, vz_max].
  void set_max_velocity(std::array<double, 3> dq_max);

  /// Set per-axis maximum acceleration.
  /// @param ddq_max Array of [ax_max, ay_max, az_max].
  void set_max_acceleration(std::array<double, 3> ddq_max);

  /// Compute trajectory step. Returns true when motion is complete.
  /// @param state Current robot state.
  /// @param time Current time in seconds.
  /// @return true when the trajectory has reached the goal.
  bool operator()(const RobotState & state, double time);

  Vector3d delta_q_d = Vector3d::Zero();  ///< Position delta output for this step

private:
  static constexpr double kMotionFinishedThreshold = 2e-3;

  bool calculate_desired_values(double t, Vector3d * delta_q_d);
  void calculate_synchronized_values();

  Vector3d q_goal_ = Vector3d::Zero();
  Vector3d q_start_ = Vector3d::Zero();
  Vector3d delta_q_ = Vector3d::Zero();
  Vector3d dq_max_sync_ = Vector3d::Zero();
  Vector3d t_1_sync_ = Vector3d::Zero();
  Vector3d t_2_sync_ = Vector3d::Zero();
  Vector3d t_f_sync_ = Vector3d::Zero();
  Vector3d q_1_ = Vector3d::Zero();
  Vector3d dq_ = Vector3d::Zero();

  double time_ = 0.0;
  double frequency_ = 10.0;

  Vector3d dq_max_ = (Vector3d() << 0.6, 0.6, 0.6).finished();
  Vector3d ddq_max_ = (Vector3d() << 0.2, 0.2, 0.2).finished();
};

}  // namespace drone
