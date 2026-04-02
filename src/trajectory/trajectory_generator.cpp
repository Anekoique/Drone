#include "drone/trajectory/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace drone
{

TrajectoryGenerator::TrajectoryGenerator(double speed_factor, std::array<double, 3> q_goal)
: q_goal_(q_goal.data())
{
  dq_max_ *= speed_factor;
  ddq_max_ *= speed_factor;
}

bool TrajectoryGenerator::calculate_desired_values(double t, Vector3d * out)
{
  using Vector3i = Eigen::Matrix<int, 3, 1, Eigen::ColMajor>;
  Vector3i sign = delta_q_.cwiseSign().cast<int>();
  Vector3d t_d = t_2_sync_ - t_1_sync_;
  std::array<bool, 3> finished{};
  dq_.setZero();

  for (int i = 0; i < 3; i++) {
    if (std::abs(delta_q_[i]) < kMotionFinishedThreshold) {
      (*out)[i] = 0;
      finished[i] = true;
    } else if (t < t_1_sync_[i]) {
      (*out)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign[i] *
                  (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      dq_[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign[i] *
               (2.0 * t - 3.0 * t_1_sync_[i]) * std::pow(t, 2.0);
    } else if (t < t_2_sync_[i]) {
      (*out)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign[i];
      dq_[i] = dq_max_sync_[i];
    } else if (t < t_f_sync_[i]) {
      double dt = t - t_1_sync_[i] - t_d[i];
      (*out)[i] = delta_q_[i] + 0.5 *
                                  (1.0 / std::pow(t_1_sync_[i], 3.0) *
                                     (t - 3.0 * t_1_sync_[i] - t_d[i]) * std::pow(dt, 3.0) +
                                   (2.0 * t - 3.0 * t_1_sync_[i] - 2.0 * t_d[i])) *
                                  dq_max_sync_[i] * sign[i];
      dq_[i] = (1.0 / std::pow(t_1_sync_[i], 3.0) * (2.0 * t - 5.0 * t_1_sync_[i] - 2.0 * t_d[i]) *
                  std::pow(dt, 2.0) +
                1.0) *
               dq_max_sync_[i] * sign[i];
    } else {
      (*out)[i] = delta_q_[i];
      finished[i] = true;
    }
  }

  return std::all_of(finished.begin(), finished.end(), [](bool x) { return x; });
}

void TrajectoryGenerator::calculate_synchronized_values()
{
  Vector3d dq_reach(dq_max_);
  Vector3d t_f = Vector3d::Zero();
  Vector3d t_1 = Vector3d::Zero();
  auto sign = delta_q_.cwiseSign().cast<int>();

  for (int i = 0; i < 3; i++) {
    if (std::abs(delta_q_[i]) > kMotionFinishedThreshold) {
      if (std::abs(delta_q_[i]) < 1.5 * std::pow(dq_max_[i], 2.0) / ddq_max_[i]) {
        dq_reach[i] = std::sqrt(2.0 / 3.0 * delta_q_[i] * sign[i] * ddq_max_[i]);
      }
      t_1[i] = 1.5 * dq_reach[i] / ddq_max_[i];
      t_f[i] = t_1[i] + std::abs(delta_q_[i]) / dq_reach[i];
    } else {
      t_f[i] = 0.01;
    }
  }

  double max_tf = t_f.maxCoeff();

  for (int i = 0; i < 3; i++) {
    if (std::abs(delta_q_[i]) > kMotionFinishedThreshold) {
      double a = 1.5 * ddq_max_[i];
      double b = -max_tf * std::pow(ddq_max_[i], 2.0);
      double c = std::abs(delta_q_[i]) * std::pow(ddq_max_[i], 2.0);
      double disc = std::max(0.0, b * b - 4.0 * a * c);

      dq_max_sync_[i] = (-b - std::sqrt(disc)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_[i];
      t_f_sync_[i] = t_1_sync_[i] + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = t_f_sync_[i] - t_1_sync_[i];
      q_1_[i] = dq_max_sync_[i] * sign[i] * 0.5 * t_1_sync_[i];
    } else {
      dq_max_sync_[i] = 0.0;
      t_1_sync_[i] = 0.0;
      t_f_sync_[i] = 0.01;
      t_2_sync_[i] = 0.0;
      q_1_[i] = 0.0;
    }
  }
}

bool TrajectoryGenerator::operator()(const RobotState & state, double time)
{
  time_ = time;
  if (time_ == 0.0) {
    q_start_ = Vector3d(state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    calculate_synchronized_values();
  }
  return calculate_desired_values(time_, &delta_q_d);
}

void TrajectoryGenerator::set_max_velocity(std::array<double, 3> dq_max)
{
  dq_max_ = Eigen::Map<const Vector3d>(dq_max.data()) / frequency_;
}

void TrajectoryGenerator::set_max_acceleration(std::array<double, 3> ddq_max)
{
  ddq_max_ = Eigen::Map<const Vector3d>(ddq_max.data()) / frequency_;
}

}  // namespace drone
