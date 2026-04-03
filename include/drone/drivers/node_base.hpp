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
  explicit NodeBase(const std::string & name, const std::string & mavros_ns = "/mavros/");

  double elapsed_time();
  double start_time() const;
  void set_start_time(double t);

  bool sim_mode() const { return sim_mode_; }
  bool debug_mode() const { return debug_mode_; }
  bool print_info() const { return print_info_; }
  bool fast_mode() const { return fast_mode_; }

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
