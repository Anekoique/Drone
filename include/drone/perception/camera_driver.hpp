#pragma once

#include <opencv2/videoio.hpp>

#include <optional>

namespace drone
{

/// Direct camera capture via V4L2/OpenCV. No ROS topic overhead.
class CameraDriver
{
public:
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
