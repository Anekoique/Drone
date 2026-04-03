// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/srv/command_long.hpp>

#include <string>
#include <unordered_map>

namespace drone
{

/// Servo control via MAVROS MAV_CMD_DO_SET_SERVO.
class Servo
{
public:
  /// @param node ROS 2 node for creating service clients and timers.
  /// @param mavros_ns MAVROS namespace.
  Servo(rclcpp::Node & node, const std::string & mavros_ns);

  /// Set servo PWM position.
  /// @param servo_number Servo output channel number.
  /// @param pwm PWM value in microseconds.
  /// @return true if the command was sent.
  bool set_servo(int servo_number, float pwm);

  /// Non-blocking fire: open servo, then close after delay via one-shot timer.
  /// Safe for concurrent calls with different servo IDs.
  /// @param servo_id Servo output channel.
  /// @param open_pwm PWM for open position.
  /// @param close_pwm PWM for closed position.
  /// @param delay_sec Delay before closing (seconds).
  void fire_servo(
    int servo_id, float open_pwm = 1864, float close_pwm = 1200, double delay_sec = 0.5);

private:
  rclcpp::Node & node_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
  std::unordered_map<int, rclcpp::TimerBase::SharedPtr> pending_timers_;
};

}  // namespace drone
