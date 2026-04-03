// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <string>

namespace drone::control
{

/// Encapsulates all MAVROS setpoint publishing.
/// Ports all 6 legacy publishers from PosControl.
class MavrosCommander
{
public:
  /// @param node ROS 2 node for creating publishers.
  /// @param ns MAVROS namespace (e.g. "/mavros/").
  MavrosCommander(rclcpp::Node & node, const std::string & ns);
  MavrosCommander() = delete;

  /// Send a velocity command (vx, vy, vz, yaw_rate).
  void send_velocity(const Eigen::Vector4f & vel);

  /// Send a position setpoint with yaw.
  void send_position(double x, double y, double z, double yaw);

  /// Send a position setpoint from a 4-vector (x, y, z, yaw).
  void send_position(const Eigen::Vector4f & pos_yaw);

  /// Send an acceleration command (ax, ay, az, yaw_accel).
  void send_acceleration(const Eigen::Vector4f & accel);

  /// Send an attitude + thrust command (roll, pitch, yaw, thrust).
  void send_attitude(const Eigen::Vector4f & attitude_thrust);

  /// Publish combined position + velocity setpoint (SET_POSITION_TARGET_LOCAL_NED).
  void publish_setpoint_raw(const Eigen::Vector4f & pos, const Eigen::Vector4f & vel);

  /// Publish a global position setpoint (lat, lon, alt, yaw).
  void publish_setpoint_raw_global(double lat, double lon, double alt, float yaw);

  /// Timed velocity: sends vel for duration seconds, returns true when done.
  /// @param vel Velocity command (vx, vy, vz, yaw_rate).
  /// @param duration_sec Duration in seconds.
  /// @return true when the duration has elapsed.
  bool send_velocity_timed(const Eigen::Vector4f & vel, double duration_sec);

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_local_pub_;
  rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr raw_global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;

  // State for send_velocity_timed (replaces static locals)
  rclcpp::Time timed_start_{0, 0, RCL_ROS_TIME};
  bool timed_active_ = false;
};

}  // namespace drone::control
