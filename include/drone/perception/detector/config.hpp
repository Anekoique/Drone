// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

/// TensorRT YOLO detector configuration.
/// All values are constexpr -- no runtime macros.

namespace drone::detector
{

// --- Tensor names ---
constexpr const char * kInputTensorName = "data";
constexpr const char * kOutputTensorName = "prob";

// --- Model parameters ---
constexpr int kNumClass = 1;             ///< Number of detection classes
constexpr int kBatchSize = 1;            ///< Inference batch size
constexpr int kInputH = 480;             ///< Network input height (pixels)
constexpr int kInputW = 640;             ///< Network input width (pixels)
constexpr int kMaxNumOutputBbox = 1000;  ///< Maximum output bounding boxes
constexpr int kNumAnchor = 3;            ///< Anchors per detection layer
constexpr float kIgnoreThresh = 0.1f;    ///< Objectness threshold for YOLO layer

// --- NMS parameters ---
constexpr float kNmsThresh = 0.45f;  ///< Non-maximum suppression IoU threshold
constexpr float kConfThresh = 0.5f;  ///< Confidence threshold for output filtering

// --- GPU ---
constexpr int kGpuId = 0;                        ///< CUDA device index
constexpr int kMaxInputImageSize = 4096 * 3112;  ///< Max pixels for preprocess buffer

}  // namespace drone::detector
