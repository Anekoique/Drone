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
  Gimbal(rclcpp::Node & node, const std::string & mavros_ns);

  /// Set gimbal angles in degrees.
  void set_gimbal(float pitch, float roll, float yaw);

  double gimbal_pitch() const { return pitch_; }
  double gimbal_roll() const { return roll_; }
  double gimbal_yaw() const { return yaw_; }

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<mavros_msgs::msg::MountControl>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;

  double pitch_ = 0, roll_ = 0, yaw_ = 0;
};

}  // namespace drone
