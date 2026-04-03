// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/perception/detector/types.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace drone::detector
{

/// Convert normalized bbox coordinates to an OpenCV Rect on the original image.
/// @param img Original image (used for size clamping).
/// @param bbox Array of 4 floats: [center_x, center_y, w, h] in network coords.
/// @return Clamped rectangle in pixel coordinates.
cv::Rect get_rect(const cv::Mat & img, float bbox[4]);

/// Apply non-maximum suppression to raw TensorRT output.
/// @param res Output vector of surviving detections.
/// @param output Raw network output buffer (flat float array).
/// @param conf_thresh Minimum confidence to keep.
/// @param nms_thresh IoU threshold for suppression.
void nms(
  std::vector<RawDetection> & res, float * output, float conf_thresh, float nms_thresh = 0.5f);

}  // namespace drone::detector
