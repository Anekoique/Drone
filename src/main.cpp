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
