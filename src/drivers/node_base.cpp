// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file node_base.cpp
/// @brief Base ROS2 node with common parameters (sim_mode, debug_mode, etc.)
///        and startup timestamp tracking.

#include "drone/drivers/node_base.hpp"

namespace drone
{

NodeBase::NodeBase(const std::string & name, const std::string & mavros_ns)
: Node(name), mode_client_(this->create_client<mavros_msgs::srv::SetMode>(mavros_ns + "set_mode"))
{
  this->declare_parameter("sim_mode", false);
  this->declare_parameter("debug_mode", false);
  this->declare_parameter("print_info", false);
  this->declare_parameter("fast_mode", false);

  this->get_parameter("sim_mode", sim_mode_);
  this->get_parameter("debug_mode", debug_mode_);
  this->get_parameter("print_info", print_info_);
  this->get_parameter("fast_mode", fast_mode_);

  timestamp_init_ = this->get_clock()->now().seconds();
}

double NodeBase::elapsed_time()
{
  return this->get_clock()->now().seconds() - timestamp_init_;
}

double NodeBase::start_time() const
{
  return start_time_;
}

void NodeBase::set_start_time(double t)
{
  start_time_ = t;
}

}  // namespace drone
