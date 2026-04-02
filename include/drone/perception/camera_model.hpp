#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <optional>

namespace drone
{

/// Pure geometry camera model. Handles pixel↔world transforms with distortion.
/// No ROS dependencies. Coordinate frames: ENU (world), NED (MAVLink), ESD (camera).
class CameraModel
{
public:
  CameraModel() = default;

  // --- Intrinsics ---
  void set_intrinsics(double fx, double fy, double cx, double cy, double width, double height);
  void set_distortion(double k1, double k2, double p1, double p2, double k3);

  // --- Extrinsics (updated per frame) ---
  void set_relative_position(const Eigen::Vector3d & offset);
  void set_relative_rotation(const Eigen::Vector3d & rotation);
  void set_parent_pose(const Eigen::Vector3d & position, const Eigen::Vector3d & rotation);

  /// Camera world position (parent + rotated offset).
  Eigen::Vector3d get_position() const;

  // --- Transforms ---

  /// Undistort a full frame using precomputed remap.
  cv::Mat undistort(const cv::Mat & frame) const;

  /// Pixel → normalized coords with distortion correction.
  Eigen::Vector2d pixel_to_normalized(const Eigen::Vector2d & pixel) const;

  /// World point → pixel coords. Returns nullopt if behind camera or out of frame.
  std::optional<Eigen::Vector2d> world_to_pixel(const Eigen::Vector3d & world_point) const;

  /// Pixel → world position at given object height (ray-plane intersection).
  std::optional<Eigen::Vector3d> pixel_to_world(
    const Eigen::Vector2d & pixel, double object_height = 0.0) const;

  /// Calculate pixel radius of a world-space sphere at given center.
  double calculate_pixel_radius(const Eigen::Vector3d & center, double world_radius) const;

  /// Calculate real-world diameter from pixel diameter and distance.
  double calculate_real_diameter(double pixel_diameter, double distance) const;

  double fx() const { return fx_; }
  double fy() const { return fy_; }

private:
  // Rotation matrix: world (ENU) → camera
  Eigen::Matrix3d rotation_world_to_camera() const;
  Eigen::Matrix3d rotation_camera_to_world() const;
  Eigen::Vector2d apply_distortion(const Eigen::Vector2d & point) const;

  // Coordinate frame conversion matrices
  static const Eigen::Matrix3d kENU2NED;
  static const Eigen::Matrix3d kNED2ENU;
  static const Eigen::Matrix3d kESD2NED;
  static const Eigen::Matrix3d kNED2ESD;

  // Intrinsics
  double fx_ = 1, fy_ = 1, cx_ = 0, cy_ = 0;
  double k1_ = 0, k2_ = 0, p1_ = 0, p2_ = 0, k3_ = 0;
  double width_ = 640, height_ = 480;

  // Extrinsics
  Eigen::Vector3d relative_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d relative_rotation_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d parent_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d parent_rotation_ = Eigen::Vector3d::Zero();
};

}  // namespace drone
