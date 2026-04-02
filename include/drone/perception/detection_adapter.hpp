#pragma once

#include "drone/perception/detection_types.hpp"

#include <rclcpp/time.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <vector>

namespace drone
{

/// Convert internal detections to standard ROS 2 Detection2DArray message.
vision_msgs::msg::Detection2DArray to_detection2d_array(
  const std::vector<Detection> & detections, const rclcpp::Time & stamp,
  const std::string & frame_id = "camera_frame");

}  // namespace drone
