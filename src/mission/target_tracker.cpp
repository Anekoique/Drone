// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file target_tracker.cpp
/// @brief Fuses raw detections with Kalman-filtered pixel tracking, projects
///        detections to world coordinates via the camera model, and clusters
///        targets for airdrop ordering. Also implements catch_target visual servo.

#include "drone/mission/target_tracker.hpp"

#include "drone/mission/frame_transforms.hpp"

#include <algorithm>
#include <cmath>

namespace drone::mission
{

TargetTracker::TargetTracker(InertialNav & inav, Gimbal & gimbal, const MissionConfig & config)
: inav_(inav), gimbal_(gimbal), config_(config)
{
  // Load camera intrinsics from Phase 0 camera.yaml values
  // These are set once; extrinsics are updated per frame
  camera_model_.set_intrinsics(611.4, 610.7, 637.2, 367.9, 1280, 720);
  camera_model_.set_distortion(0.108, -0.137, -0.001, 0.001, 0.050);
  camera_model_.set_relative_position(
    Eigen::Vector3d(
      config.drone_to_camera.x(), config.drone_to_camera.y(), config.drone_to_camera.z()));
  camera_model_.set_relative_rotation(Eigen::Vector3d(0, 0, 0));
}

/// Synchronize the camera model extrinsics with the current drone pose.
/// Uses yaw-only rotation (roll=0, pitch=0) for downward-facing camera.
void TargetTracker::update_camera_pose()
{
  Eigen::Vector3f pos = inav_.position();
  float yaw = inav_.yaw();

  // Parent pose: drone position (ENU) and rotation (roll, -pitch, yaw)
  // Legacy sign convention: pitch is negated for camera model
  camera_model_.set_parent_pose(
    Eigen::Vector3d(pos.x(), pos.y(), pos.z()),
    Eigen::Vector3d(0, 0, yaw));  // Simplified: roll=0, pitch=0 for downward camera
}

/// Ingest raw detections into per-class Kalman filters and build clustered
/// world-space target list. Each circle detection is back-projected to world
/// coordinates and its real diameter estimated from camera distance.
void TargetTracker::feed_detections(
  const std::vector<Detection> & circles, const std::vector<Detection> & h_targets, float dt)
{
  circle_filter_.update(
    circles, dt, static_cast<int>(camera_model_.width()), static_cast<int>(camera_model_.height()));
  h_filter_.update(
    h_targets, dt, static_cast<int>(camera_model_.width()),
    static_cast<int>(camera_model_.height()));

  // Build target samples from circle detections for clustering
  std::vector<TargetSample> samples;
  for (const auto & det : circles) {
    auto world_pos =
      camera_model_.pixel_to_world(Eigen::Vector2d(det.cx, det.cy), config_.bucket_height);
    if (world_pos) {
      double avg_size = (det.w + det.h) / 2.0;
      double distance = camera_model_.get_position().z() - config_.bucket_height;
      double diameter = camera_model_.calculate_real_diameter(avg_size, std::max(distance, 0.1));
      samples.push_back({*world_pos, diameter});
    }
  }

  if (!samples.empty()) {
    clustered_targets_ = find_cluster_centers(samples);
    // Sort by diameter preference
    if (config_.prefer_large_target) {
      std::sort(
        clustered_targets_.begin(), clustered_targets_.end(),
        [](const Target & a, const Target & b) { return a.diameter > b.diameter; });
    } else {
      std::sort(
        clustered_targets_.begin(), clustered_targets_.end(),
        [](const Target & a, const Target & b) { return a.diameter < b.diameter; });
    }
  }
}

bool TargetTracker::has_target(TargetClass target_class) const
{
  if (target_class == TargetClass::kCircle) return circle_filter_.has_target(target_class);
  if (target_class == TargetClass::kH) return h_filter_.has_target(target_class);
  return false;
}

std::optional<Eigen::Vector2f> TargetTracker::get_filtered_pixel(TargetClass target_class) const
{
  const auto & filter = (target_class == TargetClass::kH) ? h_filter_ : circle_filter_;
  auto pos = filter.get_filtered(target_class);
  if (!pos) return std::nullopt;
  return Eigen::Vector2f(static_cast<float>(pos->x()), static_cast<float>(pos->y()));
}

std::optional<Eigen::Vector3f> TargetTracker::pixel_to_world(
  float px, float py, float target_height) const
{
  auto world = camera_model_.pixel_to_world(Eigen::Vector2d(px, py), target_height);
  if (!world) return std::nullopt;
  return Eigen::Vector3f(
    static_cast<float>(world->x()), static_cast<float>(world->y()), static_cast<float>(world->z()));
}

// --- catch_target ---

/// Visual servo: move the drone so that the tracked target pixel aligns with
/// tar_pixel in the camera frame. Pixel coordinates are rotated by drone yaw
/// to camera-frame, normalized by the largest frame dimension, and fed to the
/// trajectory controller. Returns true when pixel error is within accuracy.
bool catch_target(
  Subsystems & subs, TargetTracker & tracker, TargetClass target_class,
  const Eigen::Vector2f & tar_pixel, float tar_z, float tar_yaw, float accuracy,
  const PID::Defaults & /*pid_defaults*/, const control::Limits & /*limits*/)
{
  auto filtered = tracker.get_filtered_pixel(target_class);
  if (!filtered) return false;

  float now_x = filtered->x();
  float now_y = tracker.frame_height() - filtered->y();  // Flip y: image y-down to y-up

  float tx = tar_pixel.x();
  float ty = tracker.frame_height() - tar_pixel.y();

  // Rotate from camera frame to world frame using drone yaw
  float yaw = subs.inav.yaw();
  auto now_rot = rotate_xy(now_x, now_y, -yaw);
  auto tar_rot = rotate_xy(tx, ty, -yaw);

  // Normalize by max frame dimension
  float max_frame = std::max(tracker.frame_width(), tracker.frame_height());

  // Use PosControl trajectory with pixel-normalized coordinates
  Eigen::Vector4f current(
    tar_rot.x() / max_frame, tar_rot.y() / max_frame, subs.inav.position().z(), subs.inav.yaw());
  Eigen::Vector4f target(now_rot.x() / max_frame, now_rot.y() / max_frame, tar_z, tar_yaw);

  subs.pos_control.trajectory_to(target);

  // Check convergence in pixel space
  return std::abs(now_x - tx) <= accuracy && std::abs(now_y - ty) <= accuracy;
}

}  // namespace drone::mission
