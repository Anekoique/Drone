// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file gimbal.cpp
/// @brief MAVROS gimbal driver. Publishes mount control commands and subscribes
///        to mount status for current gimbal orientation feedback.

#include "drone/drivers/gimbal.hpp"

namespace drone
{

Gimbal::Gimbal(rclcpp::Node & node, const std::string & mavros_ns) : node_(node)
{
  pub_ =
    node_.create_publisher<mavros_msgs::msg::MountControl>(mavros_ns + "mount_control/command", 10);

  sub_ = node_.create_subscription<geometry_msgs::msg::Vector3Stamped>(
    mavros_ns + "mount_control/status", 10,
    [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
      pitch_ = msg->vector.x;
      roll_ = msg->vector.y;
      yaw_ = msg->vector.z;
    });
}

void Gimbal::set_gimbal(float pitch, float roll, float yaw)
{
  mavros_msgs::msg::MountControl msg;
  msg.mode = 2;  // MAV_MOUNT_MODE_MAVLINK_TARGETING
  msg.pitch = pitch;
  msg.roll = roll;
  msg.yaw = yaw;
  pub_->publish(msg);
}

}  // namespace drone
