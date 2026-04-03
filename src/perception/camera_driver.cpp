// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file camera_driver.cpp
/// @brief V4L2 camera capture driver. Opens the device with MJPEG fourcc
///        at the specified resolution and provides single-frame capture.

#include "drone/perception/camera_driver.hpp"

namespace drone
{

CameraDriver::CameraDriver(int device_id, int width, int height) : width_(width), height_(height)
{
  cap_.open(device_id, cv::CAP_V4L2);
  if (cap_.isOpened()) {
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  }
}

std::optional<cv::Mat> CameraDriver::capture()
{
  if (!cap_.isOpened()) return std::nullopt;
  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty()) return std::nullopt;
  return frame;
}

}  // namespace drone
