#pragma once

#include "drone/perception/detection_types.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace drone
{

#ifdef DRONE_HAS_CUDA

/// TensorRT YOLO detector. Loads pre-built engine files and runs GPU inference.
/// Supports dual-model setup (circle + H) matching the cv_py architecture.
/// Uses pimpl to keep CUDA/TensorRT headers out of the public API.
class Detector
{
public:
  /// Load one or two TensorRT engine files.
  /// If h_engine is empty, only circle model is used.
  Detector(const std::string & circle_engine, const std::string & h_engine = "");
  ~Detector();

  Detector(const Detector &) = delete;
  Detector & operator=(const Detector &) = delete;
  Detector(Detector &&) noexcept;
  Detector & operator=(Detector &&) noexcept;

  /// Run inference on a BGR frame. Returns all detections from both models.
  std::vector<Detection> detect(const cv::Mat & frame);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#else

/// Stub detector when CUDA is not available.
class Detector
{
public:
  Detector(const std::string & /*circle_engine*/, const std::string & /*h_engine*/ = "")
  {
    throw std::runtime_error("Detector requires CUDA/TensorRT (compile with DRONE_HAS_CUDA)");
  }

  std::vector<Detection> detect(const cv::Mat & /*frame*/) { return {}; }
};

#endif  // DRONE_HAS_CUDA

}  // namespace drone
