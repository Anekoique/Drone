// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/mount_control.hpp>

#include <string>

namespace drone
{

/// Camera gimbal control via MAVROS mount_control.
/// Note: uses mavros_msgs::msg::MountControl (legacy MAVROS interface).
class Gimbal
{
public:
  /// @param node ROS 2 node for creating publisher and subscriber.
  /// @param mavros_ns MAVROS namespace.
  Gimbal(rclcpp::Node & node, const std::string & mavros_ns);

  /// Set gimbal angles in degrees.
  /// @param pitch Pitch angle (degrees, negative = down).
  /// @param roll Roll angle (degrees).
  /// @param yaw Yaw angle (degrees).
  void set_gimbal(float pitch, float roll, float yaw);

  /// Current gimbal pitch feedback (degrees).
  double gimbal_pitch() const { return pitch_; }
  /// Current gimbal roll feedback (degrees).
  double gimbal_roll() const { return roll_; }
  /// Current gimbal yaw feedback (degrees).
  double gimbal_yaw() const { return yaw_; }

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<mavros_msgs::msg::MountControl>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;

  double pitch_ = 0, roll_ = 0, yaw_ = 0;
};

}  // namespace drone
