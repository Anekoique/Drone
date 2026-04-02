#include "drone/perception/camera_model.hpp"

#include <cmath>

namespace drone
{

// Coordinate frame conversion matrices (static)
// clang-format off
const Eigen::Matrix3d CameraModel::kENU2NED = (Eigen::Matrix3d() <<
  0, 1, 0,
  1, 0, 0,
  0, 0, -1).finished();

const Eigen::Matrix3d CameraModel::kNED2ENU = CameraModel::kENU2NED.transpose();

const Eigen::Matrix3d CameraModel::kESD2NED = (Eigen::Matrix3d() <<
  0, -1, 0,
  1,  0, 0,
  0,  0, 1).finished();

const Eigen::Matrix3d CameraModel::kNED2ESD = CameraModel::kESD2NED.transpose();
// clang-format on

void CameraModel::set_intrinsics(
  double fx, double fy, double cx, double cy, double width, double height)
{
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  width_ = width;
  height_ = height;
}

void CameraModel::set_distortion(double k1, double k2, double p1, double p2, double k3)
{
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
  k3_ = k3;
}

void CameraModel::set_relative_position(const Eigen::Vector3d & offset)
{
  relative_position_ = offset;
}

void CameraModel::set_relative_rotation(const Eigen::Vector3d & rotation)
{
  relative_rotation_ = rotation;
}

void CameraModel::set_parent_pose(
  const Eigen::Vector3d & position, const Eigen::Vector3d & rotation)
{
  parent_position_ = position;
  parent_rotation_ = rotation;
}

// Note: roll matrix uses sin(roll) at (1,2) and -sin(roll) at (2,1).
// This matches the legacy CameraGimbal.h convention used in production.
// Do NOT change without recalibrating the camera system.
static Eigen::Matrix3d make_rotation(const Eigen::Vector3d & rpy)
{
  double roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

  Eigen::Matrix3d R_roll;
  R_roll << 1, 0, 0, 0, cos(roll), sin(roll), 0, -sin(roll), cos(roll);

  Eigen::Matrix3d R_pitch;
  R_pitch << cos(pitch), 0, -sin(pitch), 0, 1, 0, sin(pitch), 0, cos(pitch);

  Eigen::Matrix3d R_yaw;
  R_yaw << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;

  return R_yaw * R_pitch * R_roll;
}

Eigen::Vector3d CameraModel::get_position() const
{
  // Rotate camera offset by parent (drone) rotation only, not compound rotation
  Eigen::Matrix3d R_parent = make_rotation(parent_rotation_);
  return kNED2ENU * R_parent * kENU2NED * relative_position_ + parent_position_;
}

Eigen::Matrix3d CameraModel::rotation_world_to_camera() const
{
  Eigen::Matrix3d R_camera = make_rotation(relative_rotation_);
  Eigen::Matrix3d R_parent = make_rotation(parent_rotation_);
  return R_parent * R_camera;
}

Eigen::Matrix3d CameraModel::rotation_camera_to_world() const
{
  return rotation_world_to_camera().transpose();
}

Eigen::Vector2d CameraModel::pixel_to_normalized(const Eigen::Vector2d & pixel) const
{
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
  cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);

  cv::Point2f src_pt(static_cast<float>(pixel.x()), static_cast<float>(pixel.y()));
  std::vector<cv::Point2f> src = {src_pt}, dst;
  cv::undistortPoints(src, dst, camera_matrix, dist_coeffs);

  return Eigen::Vector2d(dst[0].x, dst[0].y);
}

Eigen::Vector2d CameraModel::apply_distortion(const Eigen::Vector2d & point) const
{
  double x = point.x(), y = point.y();
  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;
  double radial = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
  double tx = 2.0 * p1_ * x * y + p2_ * (r2 + 2.0 * x * x);
  double ty = p1_ * (r2 + 2.0 * y * y) + 2.0 * p2_ * x * y;
  return Eigen::Vector2d(x * radial + tx, y * radial + ty);
}

cv::Mat CameraModel::undistort(const cv::Mat & frame) const
{
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
  cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
  cv::Mat result;
  cv::undistort(frame, result, camera_matrix, dist_coeffs, camera_matrix);
  return result;
}

std::optional<Eigen::Vector2d> CameraModel::world_to_pixel(
  const Eigen::Vector3d & world_point) const
{
  Eigen::Vector3d relative = world_point - get_position();
  Eigen::Matrix3d R = rotation_world_to_camera();
  Eigen::Vector3d cam_point = kNED2ESD * R * kENU2NED * relative;

  if (cam_point.z() <= 0) return std::nullopt;  // behind camera

  Eigen::Vector2d norm(cam_point.x() / cam_point.z(), cam_point.y() / cam_point.z());
  Eigen::Vector2d distorted = apply_distortion(norm);

  Eigen::Vector2d pixel;
  pixel.x() = fx_ * distorted.x() + cx_;
  pixel.y() = fy_ * distorted.y() + cy_;

  if (pixel.x() >= 0 && pixel.x() < width_ && pixel.y() >= 0 && pixel.y() < height_) {
    return pixel;
  }
  return std::nullopt;  // out of frame
}

std::optional<Eigen::Vector3d> CameraModel::pixel_to_world(
  const Eigen::Vector2d & pixel, double object_height) const
{
  Eigen::Vector2d norm = pixel_to_normalized(pixel);
  Eigen::Matrix3d R = rotation_world_to_camera();

  // Ray in camera coords → NED → ENU world
  Eigen::Vector3d ray_cam(norm.x(), norm.y(), 1.0);
  ray_cam = kESD2NED * ray_cam;
  Eigen::Vector3d ray_world = kNED2ENU * R.transpose() * ray_cam;
  ray_world.normalize();

  // Ray-plane intersection at z = object_height
  if (std::abs(ray_world.z()) < 1e-6) return std::nullopt;  // parallel

  double s = (object_height - get_position().z()) / ray_world.z();
  if (s < 0) return std::nullopt;  // behind camera

  return get_position() + s * ray_world;
}

double CameraModel::calculate_pixel_radius(
  const Eigen::Vector3d & center, double world_radius) const
{
  Eigen::Vector3d relative = center - get_position();
  Eigen::Matrix3d R = rotation_world_to_camera();
  Eigen::Vector3d cam_point = kNED2ESD * R * kENU2NED * relative;
  if (cam_point.z() <= 0) return 0.0;
  return std::max((world_radius / cam_point.z()) * fx_, 5.0);
}

double CameraModel::calculate_real_diameter(double pixel_diameter, double distance) const
{
  if (fx_ <= 0.01) return 0.0;
  return (pixel_diameter * distance) / fx_;
}

}  // namespace drone
