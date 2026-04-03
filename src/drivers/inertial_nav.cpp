// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file inertial_nav.cpp
/// @brief Inertial navigation data aggregator. Subscribes to MAVROS odometry,
///        GPS, altitude, IMU, and rangefinder topics using sensor QoS and
///        reentrant callbacks for low-latency state updates.

#include "drone/drivers/inertial_nav.hpp"

#include <cmath>

namespace drone
{

InertialNav::InertialNav(rclcpp::Node & node, const std::string & mavros_ns)
{
  auto qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
  auto cb_group = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = cb_group;

  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    mavros_ns + "local_position/odom", qos,
    std::bind(&InertialNav::odom_callback, this, std::placeholders::_1), sub_opt);

  gps_sub_ = node.create_subscription<sensor_msgs::msg::NavSatFix>(
    mavros_ns + "global_position/global", qos,
    std::bind(&InertialNav::gps_callback, this, std::placeholders::_1), sub_opt);

  alt_sub_ = node.create_subscription<mavros_msgs::msg::Altitude>(
    mavros_ns + "altitude", qos, std::bind(&InertialNav::alt_callback, this, std::placeholders::_1),
    sub_opt);

  imu_sub_ = node.create_subscription<sensor_msgs::msg::Imu>(
    mavros_ns + "imu/data", qos, std::bind(&InertialNav::imu_callback, this, std::placeholders::_1),
    sub_opt);

  range_sub_ = node.create_subscription<sensor_msgs::msg::Range>(
    mavros_ns + "rangefinder/rangefinder", qos,
    std::bind(&InertialNav::range_callback, this, std::placeholders::_1), sub_opt);
}

void InertialNav::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  position_ = Eigen::Vector3f(
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  orientation_ = Eigen::Quaternionf(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  velocity_ = Eigen::Vector4f(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
    msg->twist.twist.angular.z);
}

void InertialNav::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  gps_ = Eigen::Vector3f(msg->latitude, msg->longitude, msg->altitude);
}

void InertialNav::alt_callback(const mavros_msgs::msg::Altitude::SharedPtr msg)
{
  altitude_ = msg->monotonic;
}

void InertialNav::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  linear_acceleration_ = Eigen::Vector3f(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  angular_velocity_ =
    Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
}

void InertialNav::range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  rangefinder_height_ = msg->range;
}

/// Extract yaw from the stored quaternion using the standard ZYX Euler conversion.
float InertialNav::yaw() const
{
  float w = orientation_.w(), x = orientation_.x();
  float y = orientation_.y(), z = orientation_.z();
  return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

}  // namespace drone
