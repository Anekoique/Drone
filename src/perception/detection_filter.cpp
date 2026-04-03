// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file detection_filter.cpp
/// @brief Per-class Kalman filtering of 2D detections. Tracks position and
///        velocity in pixel space, prioritizing the detection closest to frame center.

#include "drone/perception/detection_filter.hpp"

#include <algorithm>
#include <cmath>

namespace drone
{

// --- KalmanFilter2D ---

KalmanFilter2D::KalmanFilter2D(double process_noise, double measurement_noise)
{
  state_ = Eigen::Vector4d::Zero();
  F_ = Eigen::Matrix4d::Identity();
  H_ = Eigen::Matrix<double, 2, 4>::Zero();
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  Q_ = Eigen::Matrix4d::Identity() * process_noise;
  R_ = Eigen::Matrix2d::Identity() * measurement_noise;
  P_ = Eigen::Matrix4d::Identity();
}

/// Predict-update cycle for the 2D Kalman filter.
/// State is [x, y, vx, vy]. On first call, initializes state directly from measurement.
/// Uses Joseph-form covariance update for numerical stability.
void KalmanFilter2D::update(double x, double y, double dt)
{
  if (!initialized_) {
    state_ << x, y, 0.0, 0.0;
    initialized_ = true;
    return;
  }

  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Predict
  state_ = F_ * state_;
  P_ = F_ * P_ * F_.transpose() + Q_;

  // Update
  Eigen::Vector2d measurement(x, y);
  Eigen::Vector2d innovation = measurement - H_ * state_;
  Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();

  state_ = state_ + K * innovation;
  // Joseph form for numerical stability
  Eigen::Matrix4d IKH = Eigen::Matrix4d::Identity() - K * H_;
  P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();
}

// --- DetectionFilter ---

const std::vector<Detection> DetectionFilter::empty_detections_;

DetectionFilter::DetectionFilter(double process_noise, double measurement_noise)
: process_noise_(process_noise), measurement_noise_(measurement_noise)
{
}

/// Sort incoming detections by class, rank each class by Manhattan distance to
/// frame center, and feed the closest detection per class into its Kalman filter.
void DetectionFilter::update(
  const std::vector<Detection> & detections, double dt, int frame_width, int frame_height)
{
  // Clear previous raw detections
  raw_detections_.clear();

  // Sort detections into per-class buckets, sorted by distance to frame center
  float cx = static_cast<float>(frame_width) / 2.0f;
  float cy = static_cast<float>(frame_height) / 2.0f;

  for (const auto & det : detections) {
    raw_detections_[det.class_id].push_back(det);
  }

  // Sort each class by distance to center
  for (auto & [cls, dets] : raw_detections_) {
    std::sort(dets.begin(), dets.end(), [cx, cy](const Detection & a, const Detection & b) {
      float da = std::abs(a.cx - cx) + std::abs(a.cy - cy);
      float db = std::abs(b.cx - cx) + std::abs(b.cy - cy);
      return da < db;
    });
  }

  // Update Kalman filters for classes that have detections
  for (const auto & [cls, dets] : raw_detections_) {
    if (dets.empty()) continue;
    auto & filter = filters_[cls];
    if (!filter.initialized()) {
      filter = KalmanFilter2D(process_noise_, measurement_noise_);
    }
    filter.update(dets[0].cx, dets[0].cy, dt);
  }
}

std::optional<Eigen::Vector2d> DetectionFilter::get_filtered(TargetClass target) const
{
  auto it = filters_.find(static_cast<int>(target));
  if (it == filters_.end() || !it->second.initialized()) return std::nullopt;
  return Eigen::Vector2d(it->second.x(), it->second.y());
}

std::optional<Eigen::Vector2d> DetectionFilter::get_velocity(TargetClass target) const
{
  auto it = filters_.find(static_cast<int>(target));
  if (it == filters_.end() || !it->second.initialized()) return std::nullopt;
  return Eigen::Vector2d(it->second.vx(), it->second.vy());
}

bool DetectionFilter::has_target(TargetClass target) const
{
  auto it = raw_detections_.find(static_cast<int>(target));
  return it != raw_detections_.end() && !it->second.empty();
}

const std::vector<Detection> & DetectionFilter::get_raw(TargetClass target) const
{
  auto it = raw_detections_.find(static_cast<int>(target));
  if (it == raw_detections_.end()) return empty_detections_;
  return it->second;
}

}  // namespace drone
