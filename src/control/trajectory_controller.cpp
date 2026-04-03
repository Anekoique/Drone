#include "drone/control/trajectory_controller.hpp"

#include <cmath>

namespace drone::control
{

TrajectoryController::TrajectoryController(const Limits & limits) : limits_(limits)
{
}

std::pair<Eigen::Vector4f, bool> TrajectoryController::setpoint_world(
  const Eigen::Vector3f & pos_current, const Eigen::Vector4f & pos_target, float accuracy)
{
  // Detect target change: reinitialize if target moved
  if (setpoint_state_.initialized) {
    Eigen::Vector4f diff = pos_target - setpoint_state_.cached_target;
    if (diff.norm() > 1e-6f) {
      setpoint_state_.initialized = false;
    }
  }

  if (!setpoint_state_.initialized) {
    setpoint_state_.cached_target = pos_target;
    setpoint_state_.initialized = true;
  }

  // Check arrival
  bool done = std::abs(pos_current.x() - pos_target.x()) <= accuracy &&
              std::abs(pos_current.y() - pos_target.y()) <= accuracy &&
              std::abs(pos_current.z() - pos_target.z()) <= accuracy;

  if (done) {
    setpoint_state_.initialized = false;
  }

  return {setpoint_state_.cached_target, done};
}

std::pair<Eigen::Vector4f, bool> TrajectoryController::setpoint_relative(
  const Eigen::Vector3f & pos_current, const Eigen::Vector4f & pos_offset, float accuracy)
{
  // On first call, convert relative offset to absolute target
  if (!relative_state_.initialized) {
    relative_state_.cached_target = Eigen::Vector4f(
      pos_current.x() + pos_offset.x(), pos_current.y() + pos_offset.y(),
      pos_current.z() + pos_offset.z(), pos_offset.w());
    relative_state_.initialized = true;
  }

  // Detect offset change
  // (comparing the offset is tricky since pos_current changes; use a tolerance on the offset)

  bool done = std::abs(pos_current.x() - relative_state_.cached_target.x()) <= accuracy &&
              std::abs(pos_current.y() - relative_state_.cached_target.y()) <= accuracy &&
              std::abs(pos_current.z() - relative_state_.cached_target.z()) <= accuracy;

  if (done) {
    relative_state_.initialized = false;
  }

  return {relative_state_.cached_target, done};
}

std::pair<Eigen::Vector4f, bool> TrajectoryController::circle(
  float a, float b, float height, float angular_vel, float yaw)
{
  if (!circle_state_.active) {
    circle_state_.angle = yaw;
    circle_state_.active = true;
  }

  circle_state_.angle += angular_vel;
  float c = std::cos(circle_state_.angle);
  float s = std::sin(circle_state_.angle);

  Eigen::Vector4f waypoint(a * c, b * s, height, 0.0f);

  // Legacy never returns done (uses `if (false)` guard); match that behavior.
  return {waypoint, false};
}

std::pair<Eigen::Vector4f, bool> TrajectoryController::generator_world(
  const Eigen::Vector3f & pos_current, double speed_factor, const std::array<double, 3> & q_goal,
  float target_yaw, const Eigen::Vector3f & max_speed, const Eigen::Vector3f & max_accel)
{
  // Detect target change
  if (generator_state_.initialized) {
    Eigen::Vector4f new_target(
      static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]),
      0.0f);
    Eigen::Vector4f diff = new_target - generator_state_.cached_target;
    if (diff.norm() > 1e-6f) {
      generator_state_.initialized = false;
    }
  }

  if (!generator_state_.initialized) {
    generator_state_.start_pos = pos_current;
    generator_state_.cached_target = Eigen::Vector4f(
      static_cast<float>(q_goal[0]), static_cast<float>(q_goal[1]), static_cast<float>(q_goal[2]),
      0.0f);
    generator_state_.count = 0;

    RobotState state;
    state.q_d = {
      static_cast<double>(pos_current.x()), static_cast<double>(pos_current.y()),
      static_cast<double>(pos_current.z())};

    generator_ = std::make_unique<TrajectoryGenerator>(speed_factor, q_goal);
    generator_->set_max_velocity(
      {std::min(static_cast<double>(limits_.speed_max_xy), static_cast<double>(max_speed.x())),
       std::min(static_cast<double>(limits_.speed_max_xy), static_cast<double>(max_speed.y())),
       std::min(static_cast<double>(limits_.speed_max_z), static_cast<double>(max_speed.z()))});
    generator_->set_max_acceleration(
      {std::min(static_cast<double>(limits_.accel_max_xy), static_cast<double>(max_accel.x())),
       std::min(static_cast<double>(limits_.accel_max_xy), static_cast<double>(max_accel.y())),
       std::min(static_cast<double>(limits_.accel_max_z), static_cast<double>(max_accel.z()))});

    generator_state_.initialized = true;
  }

  RobotState state;
  state.q_d = {
    static_cast<double>(pos_current.x()), static_cast<double>(pos_current.y()),
    static_cast<double>(pos_current.z())};

  bool finished = (*generator_)(state, static_cast<double>(generator_state_.count));
  generator_state_.count++;

  // Waypoint in relative coordinates from start, converted to PID target
  Eigen::Vector4f waypoint(
    generator_state_.start_pos.x() + static_cast<float>(generator_->delta_q_d.x()),
    generator_state_.start_pos.y() + static_cast<float>(generator_->delta_q_d.y()),
    generator_state_.start_pos.z() + static_cast<float>(generator_->delta_q_d.z()), target_yaw);

  if (finished) {
    generator_state_.initialized = false;
  }

  return {waypoint, finished};
}

void TrajectoryController::reset_setpoint()
{
  setpoint_state_ = SetpointState{};
}

void TrajectoryController::reset_relative()
{
  relative_state_ = SetpointState{};
}

void TrajectoryController::reset_circle()
{
  circle_state_ = CircleState{};
}

void TrajectoryController::reset_generator()
{
  generator_state_ = GeneratorState{};
  generator_.reset();
}

void TrajectoryController::reset_all()
{
  reset_setpoint();
  reset_relative();
  reset_circle();
  reset_generator();
}

}  // namespace drone::control
