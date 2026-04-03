// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <opencv2/videoio.hpp>

#include <optional>

namespace drone
{

/// Direct camera capture via V4L2/OpenCV. No ROS topic overhead.
class CameraDriver
{
public:
  /// Open a camera device for capture.
  /// @param device_id V4L2 device index (e.g. 0 for /dev/video0).
  /// @param width Requested frame width.
  /// @param height Requested frame height.
  explicit CameraDriver(int device_id = 0, int width = 1280, int height = 720);

  /// Capture a single frame. Returns std::nullopt if capture fails.
  std::optional<cv::Mat> capture();

  int width() const { return width_; }
  int height() const { return height_; }
  bool is_open() const { return cap_.isOpened(); }

private:
  cv::VideoCapture cap_;
  int width_;
  int height_;
};

}  // namespace drone
