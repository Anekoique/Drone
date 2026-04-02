#pragma once

#include "drone/perception/detection_types.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <chrono>
#include <optional>
#include <unordered_map>

namespace drone
{

/// Simple 2D Kalman filter for tracking a single target in pixel space.
class KalmanFilter2D
{
public:
  explicit KalmanFilter2D(double process_noise = 0.01, double measurement_noise = 0.1);

  void update(double x, double y, double dt);

  double x() const { return state_(0); }
  double y() const { return state_(1); }
  double vx() const { return state_(2); }
  double vy() const { return state_(3); }
  bool initialized() const { return initialized_; }

private:
  Eigen::Vector4d state_;
  Eigen::Matrix4d F_;              // state transition
  Eigen::Matrix<double, 2, 4> H_;  // observation
  Eigen::Matrix4d Q_;              // process noise
  Eigen::Matrix2d R_;              // measurement noise
  Eigen::Matrix4d P_;              // error covariance
  bool initialized_ = false;
};

/// Multi-target detection filter. Tracks one Kalman filter per target class.
class DetectionFilter
{
public:
  explicit DetectionFilter(double process_noise = 0.015, double measurement_noise = 0.5);

  void update(
    const std::vector<Detection> & detections, double dt, int frame_width, int frame_height);

  std::optional<Eigen::Vector2d> get_filtered(TargetClass target) const;
  std::optional<Eigen::Vector2d> get_velocity(TargetClass target) const;
  bool has_target(TargetClass target) const;

  /// Get raw detections for a target class (sorted by distance to center).
  const std::vector<Detection> & get_raw(TargetClass target) const;

private:
  std::unordered_map<int, KalmanFilter2D> filters_;
  std::unordered_map<int, std::vector<Detection>> raw_detections_;
  static const std::vector<Detection> empty_detections_;
  double process_noise_;
  double measurement_noise_;
};

}  // namespace drone
