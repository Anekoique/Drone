#pragma once

/// TensorRT YOLO detector configuration.
/// All values are constexpr — no runtime macros.

namespace drone::detector
{

// Precision mode (set at engine build time)
// USE_FP16 recommended for Jetson inference speed

// Tensor names
constexpr const char * kInputTensorName = "data";
constexpr const char * kOutputTensorName = "prob";

// Model parameters
constexpr int kNumClass = 1;
constexpr int kBatchSize = 1;
constexpr int kInputH = 480;
constexpr int kInputW = 640;
constexpr int kMaxNumOutputBbox = 1000;
constexpr int kNumAnchor = 3;
constexpr float kIgnoreThresh = 0.1f;

// NMS parameters
constexpr float kNmsThresh = 0.45f;
constexpr float kConfThresh = 0.5f;

// GPU
constexpr int kGpuId = 0;
constexpr int kMaxInputImageSize = 4096 * 3112;

}  // namespace drone::detector
