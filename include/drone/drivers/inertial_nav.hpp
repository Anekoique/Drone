// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <string>

namespace drone
{

/// Inertial navigation: subscribes to MAVROS odometry, GPS, IMU, rangefinder.
/// All fields are private with const getters.
/// Precondition: single-threaded executor (no mutex on callback data).
class InertialNav
{
public:
  /// @param node ROS 2 node for creating subscribers.
  /// @param mavros_ns MAVROS namespace.
  InertialNav(rclcpp::Node & node, const std::string & mavros_ns);

  /// Local ENU position from odometry.
  Eigen::Vector3f position() const { return position_; }
  /// Velocity (x, y, z, yaw_rate). The w() component is yaw rate.
  Eigen::Vector4f velocity() const { return velocity_; }
  /// Orientation quaternion from odometry.
  Eigen::Quaternionf orientation() const { return orientation_; }
  /// GPS coordinates (lat, lon, alt).
  Eigen::Vector3f gps() const { return gps_; }
  /// AMSL altitude from MAVROS altitude topic.
  float altitude() const { return altitude_; }
  /// IMU linear acceleration (m/s^2).
  Eigen::Vector3f linear_acceleration() const { return linear_acceleration_; }
  /// IMU angular velocity (rad/s).
  Eigen::Vector3f angular_velocity() const { return angular_velocity_; }
  /// Rangefinder height above ground (m).
  float rangefinder_height() const { return rangefinder_height_; }

  /// Yaw computed from quaternion (radians, ENU convention).
  float yaw() const;

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr alt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void alt_callback(const mavros_msgs::msg::Altitude::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);

  Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
  Eigen::Vector4f velocity_ = Eigen::Vector4f::Zero();
  Eigen::Quaternionf orientation_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f gps_ = Eigen::Vector3f::Zero();
  float altitude_ = 0;
  Eigen::Vector3f linear_acceleration_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f angular_velocity_ = Eigen::Vector3f::Zero();
  float rangefinder_height_ = 0;
};

}  // namespace drone
