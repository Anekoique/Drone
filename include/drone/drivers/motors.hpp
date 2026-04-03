// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/utils/timer.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <cstdint>
#include <string>

namespace drone
{

/// Motor control: arm, takeoff, land, mode switch, parameter set.
/// Uses MAVROS services and state subscriber.
class Motors
{
public:
  /// @param node ROS 2 node for creating clients and subscribers.
  /// @param mavros_ns MAVROS namespace.
  Motors(rclcpp::Node & node, const std::string & mavros_ns);

  /// Takeoff state machine. Call repeatedly from timer. Returns true when airborne.
  /// @param current_z Current altitude.
  /// @param takeoff_altitude Target altitude.
  /// @param yaw Heading for takeoff.
  bool takeoff(float current_z, float takeoff_altitude = 5.0f, float yaw = 0.0f);

  /// Arm or disarm the motors.
  /// @param do_arm true to arm, false to disarm.
  /// @return true if the service call was sent successfully.
  bool arm(bool do_arm);

  /// Switch flight mode (e.g. "GUIDED", "LAND").
  /// @param mode Mode string.
  /// @return true if the service call was sent successfully.
  bool switch_mode(const std::string & mode);

  /// Set home position with given yaw.
  /// @return true if the service call was sent successfully.
  bool set_home_position(float yaw = 0.0f);

  /// Send a takeoff or land command.
  /// @param cmd "takeoff" or "land".
  /// @return true if the service call was sent successfully.
  bool command_takeoff_or_land(const std::string & cmd, float altitude = 5.0f, float yaw = 0.0f);

  /// Set a MAVROS parameter value.
  /// @param name Parameter name.
  /// @param value Parameter value.
  /// @return true if the service call was sent successfully.
  bool set_param(const std::string & name, double value);

  bool armed() const { return armed_; }
  bool connected() const { return connected_; }
  bool guided() const { return guided_; }
  std::string mode() const { return mode_; }
  uint8_t system_status() const { return system_status_; }
  /// Home position in local ENU frame.
  Eigen::Vector3f home_position() const { return home_position_; }
  /// Home position in global frame (lat, lon, alt).
  Eigen::Vector3f home_position_global() const { return home_position_global_; }

  /// Takeoff state machine states.
  enum class TakeoffState
  {
    kInit,
    kWaitForCommand,
    kWaitForStable,
    kArmRequested,
    kTakeoff,
    kEnd
  };

  bool takeoff_command = false;  ///< Set externally to trigger takeoff

private:
  rclcpp::Node & node_;

  // MAVROS service clients
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr home_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr param_client_;

  // MAVROS state subscriber
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
  void home_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);

  // State
  bool armed_ = false;
  bool connected_ = false;
  bool guided_ = false;
  std::string mode_;
  uint8_t system_status_ = 0;
  Eigen::Vector3f home_position_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f home_position_global_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf home_quaternion_ = Eigen::Quaternionf::Identity();

  // Takeoff state machine
  TakeoffState takeoff_state_ = TakeoffState::kInit;
  Timer takeoff_timer_;
  int takeoff_attempts_ = 0;
};

}  // namespace drone
