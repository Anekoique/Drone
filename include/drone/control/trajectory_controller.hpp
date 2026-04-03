#pragma once

#include "drone/control/limits.hpp"
#include "drone/trajectory/trajectory_generator.hpp"

#include <Eigen/Core>

#include <memory>
#include <utility>

namespace drone::control
{

/// Per-method state structs (replaces static locals from legacy).

struct SetpointState
{
  Eigen::Vector4f cached_target = Eigen::Vector4f::Zero();
  bool initialized = false;
};

struct CircleState
{
  float angle = 0.0f;
  bool active = false;
};

struct GeneratorState
{
  Eigen::Vector3f start_pos = Eigen::Vector3f::Zero();
  Eigen::Vector4f cached_target = Eigen::Vector4f::Zero();
  bool initialized = false;
  uint64_t count = 0;
};

/// Trajectory generation: returns waypoints for PosControl to feed to VelocityController.
/// No VelocityController dependency — pure trajectory computation.
class TrajectoryController
{
public:
  explicit TrajectoryController(const Limits & limits);
  TrajectoryController() = default;

  /// S-curve trajectory: returns {waypoint_xyzw, done}.
  /// On first call (after reset), caches target and resets PID in caller.
  /// If target changes between calls, reinitializes automatically.
  std::pair<Eigen::Vector4f, bool> setpoint_world(
    const Eigen::Vector3f & pos_current, const Eigen::Vector4f & pos_target,
    float accuracy = 0.05f);

  /// Relative-frame trajectory: target is offset from pos_current on first call.
  std::pair<Eigen::Vector4f, bool> setpoint_relative(
    const Eigen::Vector3f & pos_current, const Eigen::Vector4f & pos_offset,
    float accuracy = 0.05f);

  /// Circular trajectory: returns {position_xyzw, done}.
  /// Uses setpoint_raw style (position + velocity).
  std::pair<Eigen::Vector4f, bool> circle(
    float a, float b, float height, float angular_vel, float yaw);

  /// S-curve via TrajectoryGenerator: returns {waypoint_xyzw, done}.
  std::pair<Eigen::Vector4f, bool> generator_world(
    const Eigen::Vector3f & pos_current, double speed_factor, const std::array<double, 3> & q_goal,
    float target_yaw, const Eigen::Vector3f & max_speed, const Eigen::Vector3f & max_accel);

  void reset_setpoint();
  void reset_relative();
  void reset_circle();
  void reset_generator();
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
