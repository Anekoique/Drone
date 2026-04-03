// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file main.cpp
/// @brief Application entry point. Initializes ROS2, creates the DroneNode,
///        and spins until shutdown.

#include "drone/mission/drone_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<drone::mission::DroneNode>("/mavros/", "config");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
