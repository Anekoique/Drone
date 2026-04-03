// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file mavros_commander.cpp
/// @brief MAVROS command interface. Publishes velocity, position, acceleration,
///        attitude, and raw setpoint messages to the flight controller via ROS2 topics.

#include "drone/control/mavros_commander.hpp"

#include <cmath>

namespace drone::control
{

MavrosCommander::MavrosCommander(rclcpp::Node & node, const std::string & ns) : node_(node)
{
  twist_pub_ =
    node_.create_publisher<geometry_msgs::msg::TwistStamped>(ns + "setpoint_velocity/cmd_vel", 5);
  pose_pub_ =
    node_.create_publisher<geometry_msgs::msg::PoseStamped>(ns + "setpoint_position/local", 5);
  raw_local_pub_ =
    node_.create_publisher<mavros_msgs::msg::PositionTarget>(ns + "setpoint_raw/local", 5);
  raw_global_pub_ =
    node_.create_publisher<mavros_msgs::msg::GlobalPositionTarget>(ns + "setpoint_raw/global", 5);
  accel_pub_ =
    node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(ns + "setpoint_accel/accel", 5);
  attitude_pub_ =
    node_.create_publisher<mavros_msgs::msg::AttitudeTarget>(ns + "setpoint_raw/attitude", 5);
}

void MavrosCommander::send_velocity(const Eigen::Vector4f & vel)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = vel.x();
  msg.twist.linear.y = vel.y();
  msg.twist.linear.z = vel.z();
  msg.twist.angular.z = vel.w();
  msg.header.stamp = node_.now();
  msg.header.frame_id = "base_link";
  twist_pub_->publish(msg);
}

void MavrosCommander::send_position(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = "base_link";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.orientation.w = std::cos(yaw / 2.0);
  pose_pub_->publish(msg);
}

void MavrosCommander::send_position(const Eigen::Vector4f & pos_yaw)
{
  send_position(pos_yaw.x(), pos_yaw.y(), pos_yaw.z(), pos_yaw.w());
}

void MavrosCommander::send_acceleration(const Eigen::Vector4f & accel)
{
  geometry_msgs::msg::Vector3Stamped msg;
  msg.vector.x = accel.x();
  msg.vector.y = accel.y();
  msg.vector.z = accel.z();
  msg.header.stamp = node_.now();
  accel_pub_->publish(msg);
}

void MavrosCommander::send_attitude(const Eigen::Vector4f & attitude_thrust)
{
  mavros_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = node_.now();
  msg.thrust = attitude_thrust.w();
  // Roll, pitch, yaw from x, y, z
  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = std::sin(attitude_thrust.z() / 2.0f);
  msg.orientation.w = std::cos(attitude_thrust.z() / 2.0f);
  msg.body_rate.x = attitude_thrust.x();
  msg.body_rate.y = attitude_thrust.y();
  attitude_pub_->publish(msg);
}

/// Publish combined position + velocity setpoint via setpoint_raw/local.
/// Ignores acceleration fields (used for circle/orbit with velocity feedforward).
void MavrosCommander::publish_setpoint_raw(const Eigen::Vector4f & pos, const Eigen::Vector4f & vel)
{
  mavros_msgs::msg::PositionTarget msg;
  msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
  msg.position.x = pos.x();
  msg.position.y = pos.y();
  msg.position.z = pos.z();
  msg.yaw = pos.w();
  msg.velocity.x = vel.x();
  msg.velocity.y = vel.y();
  msg.velocity.z = vel.z();
  msg.yaw_rate = vel.w();
  msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                  mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                  mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
  msg.header.stamp = node_.now();
  raw_local_pub_->publish(msg);
}

void MavrosCommander::publish_setpoint_raw_global(double lat, double lon, double alt, float yaw)
{
  mavros_msgs::msg::GlobalPositionTarget msg;
  msg.coordinate_frame = mavros_msgs::msg::GlobalPositionTarget::FRAME_GLOBAL_INT;
  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;
  msg.yaw = yaw;
  msg.header.stamp = node_.now();
  raw_global_pub_->publish(msg);
}

/// Send a constant velocity command for a fixed duration, then stop.
/// Tracks elapsed time internally; returns true when the duration has expired.
bool MavrosCommander::send_velocity_timed(const Eigen::Vector4f & vel, double duration_sec)
{
  if (!timed_active_) {
    timed_start_ = node_.now();
    timed_active_ = true;
  }

  double elapsed = (node_.now() - timed_start_).seconds();
  if (elapsed > duration_sec) {
    // Stop the drone
    send_velocity(Eigen::Vector4f::Zero());
    timed_active_ = false;
    return true;
  }

  send_velocity(vel);
  return false;
}

}  // namespace drone::control
