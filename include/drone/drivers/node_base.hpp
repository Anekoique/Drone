// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/srv/set_mode.hpp>

#include <string>

namespace drone
{

/// Base ROS 2 node for the drone mission system.
/// Declares common parameters and provides the mode switch client.
class NodeBase : public rclcpp::Node
{
public:
  /// @param name ROS 2 node name.
  /// @param mavros_ns MAVROS namespace (e.g. "/mavros/").
  explicit NodeBase(const std::string & name, const std::string & mavros_ns = "/mavros/");

  /// Seconds elapsed since start_time was set.
  double elapsed_time();

  /// @return The recorded mission start time (seconds).
  double start_time() const;

  /// Set the mission start time.
  /// @param t Start time in seconds (from ROS clock).
  void set_start_time(double t);

  /// True if running in simulation (declared parameter).
  bool sim_mode() const { return sim_mode_; }
  /// True if debug output is enabled.
  bool debug_mode() const { return debug_mode_; }
  /// True if verbose info printing is enabled.
  bool print_info() const { return print_info_; }
  /// True if fast (reduced-timeout) mode is active.
  bool fast_mode() const { return fast_mode_; }

  /// MAVROS set_mode service client.
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client() { return mode_client_; }

private:
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  bool sim_mode_ = false;
  bool debug_mode_ = false;
  bool print_info_ = false;
  bool fast_mode_ = false;
  double timestamp_init_ = 0;
  double start_time_ = 0;
};

}  // namespace drone
