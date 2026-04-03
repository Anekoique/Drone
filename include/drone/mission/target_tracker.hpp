// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/mission/mission_types.hpp"
#include "drone/perception/camera_model.hpp"
#include "drone/perception/clustering.hpp"
#include "drone/perception/detection_filter.hpp"
#include "drone/perception/detection_types.hpp"

#include <Eigen/Core>

#include <optional>
#include <vector>

namespace drone::mission
{

/// Bridges detector output to mission-level target tracking.
/// Processes raw detections through Kalman filtering, pixel-to-world transforms,
/// and clustering to produce world-frame target positions.
class TargetTracker
{
public:
  /// @param inav Inertial nav for drone pose.
  /// @param gimbal Gimbal for camera orientation.
  /// @param config Mission config for camera intrinsics and offsets.
  TargetTracker(InertialNav & inav, Gimbal & gimbal, const MissionConfig & config);

  /// Update camera model with current drone pose. Call each tick.
  void update_camera_pose();

  /// Feed raw detections from the detector. Call after inference.
  /// @param circles Circle-class detections.
  /// @param h_targets H-class detections.
  /// @param dt Time step in seconds.
  void feed_detections(
    const std::vector<Detection> & circles, const std::vector<Detection> & h_targets, float dt);

  /// Get clustered airdrop targets in world frame, sorted by diameter.
  std::vector<Target> get_clustered_targets() const { return clustered_targets_; }

  /// Check if a specific target class is currently detected.
  bool has_target(TargetClass target_class) const;

  /// Get filtered pixel position for a target class.
  /// @return Pixel (x, y) or nullopt if not tracked.
  std::optional<Eigen::Vector2f> get_filtered_pixel(TargetClass target_class) const;

  /// Convert pixel to world position at given height.
  /// @param px Pixel X.
  /// @param py Pixel Y.
  /// @param target_height Object height in world frame.
  /// @return World position or nullopt if projection fails.
  std::optional<Eigen::Vector3f> pixel_to_world(float px, float py, float target_height) const;

  float frame_width() const { return static_cast<float>(camera_model_.width()); }
  float frame_height() const { return static_cast<float>(camera_model_.height()); }

private:
  InertialNav & inav_;
  Gimbal & gimbal_;
  const MissionConfig & config_;

  CameraModel camera_model_;
  DetectionFilter circle_filter_;
  DetectionFilter h_filter_;
  std::vector<Target> clustered_targets_;
};

/// Visual servo: move drone to center a detected target in the camera frame.
/// @param subs Hardware subsystem references.
/// @param tracker Target tracker for filtered detections.
/// @param target_class Which target class to servo on.
/// @param tar_pixel Target pixel position to center on.
/// @param tar_z Target altitude for approach.
/// @param tar_yaw Target yaw during servo.
/// @param accuracy Pixel distance threshold.
/// @param pid_defaults PID gains for visual servo.
/// @param limits Speed limits during servo.
/// @return true when target pixel is within accuracy of tar_pixel.
bool catch_target(
  Subsystems & subs, TargetTracker & tracker, TargetClass target_class,
  const Eigen::Vector2f & tar_pixel, float tar_z, float tar_yaw, float accuracy,
  const PID::Defaults & pid_defaults, const control::Limits & limits);

}  // namespace drone::mission
