// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/perception/detector/config.hpp"

namespace drone::detector
{

/// YOLO anchor kernel definition for a single detection layer.
struct YoloKernel
{
  int width;                      ///< Feature map width
  int height;                     ///< Feature map height
  float anchors[kNumAnchor * 2];  ///< Anchor (w,h) pairs
};

/// Raw TensorRT detection output (GPU format).
struct alignas(float) RawDetection
{
  float bbox[4];   ///< Bounding box: center_x, center_y, w, h
  float conf;      ///< Combined confidence (bbox_conf * cls_conf)
  float class_id;  ///< Predicted class index
  float mask[32];  ///< Segmentation mask coefficients (unused for detection-only)
};

}  // namespace drone::detector
