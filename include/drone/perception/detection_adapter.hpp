// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/perception/detection_types.hpp"

#include <rclcpp/time.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

#include <vector>

namespace drone
{

/// Convert internal detections to standard ROS 2 Detection2DArray message.
/// @param detections Internal detection list to convert.
/// @param stamp ROS timestamp for the output message header.
/// @param frame_id TF frame ID for the output message header.
/// @return Populated Detection2DArray message.
vision_msgs::msg::Detection2DArray to_detection2d_array(
  const std::vector<Detection> & detections, const rclcpp::Time & stamp,
  const std::string & frame_id = "camera_frame");

}  // namespace drone
