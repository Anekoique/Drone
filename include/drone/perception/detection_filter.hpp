// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

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
/// State vector: [x, y, vx, vy].
class KalmanFilter2D
{
public:
  /// @param process_noise Process noise covariance scale.
  /// @param measurement_noise Measurement noise covariance scale.
  explicit KalmanFilter2D(double process_noise = 0.01, double measurement_noise = 0.1);

  /// Feed a new measurement and advance the filter.
  /// @param x Measured pixel X.
  /// @param y Measured pixel Y.
  /// @param dt Time step in seconds.
  void update(double x, double y, double dt);

  /// Filtered pixel X position.
  double x() const { return state_(0); }
  /// Filtered pixel Y position.
  double y() const { return state_(1); }
  /// Estimated pixel X velocity.
  double vx() const { return state_(2); }
  /// Estimated pixel Y velocity.
  double vy() const { return state_(3); }
  /// True after the first measurement has been processed.
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
  /// @param process_noise Kalman process noise scale.
  /// @param measurement_noise Kalman measurement noise scale.
  explicit DetectionFilter(double process_noise = 0.015, double measurement_noise = 0.5);

  /// Update filters with new detections for this frame.
  /// @param detections Raw detections from the detector.
  /// @param dt Time step in seconds.
  /// @param frame_width Image width for center-distance sorting.
  /// @param frame_height Image height for center-distance sorting.
  void update(
    const std::vector<Detection> & detections, double dt, int frame_width, int frame_height);

  /// Get filtered pixel position for a target class.
  /// @return Filtered (x, y) or nullopt if not tracked.
  std::optional<Eigen::Vector2d> get_filtered(TargetClass target) const;

  /// Get estimated pixel velocity for a target class.
  /// @return Velocity (vx, vy) or nullopt if not tracked.
  std::optional<Eigen::Vector2d> get_velocity(TargetClass target) const;

  /// @return true if the given target class is currently being tracked.
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
