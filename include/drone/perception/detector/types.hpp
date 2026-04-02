#pragma once

#include "drone/perception/detector/config.hpp"

namespace drone::detector
{

struct YoloKernel
{
  int width;
  int height;
  float anchors[kNumAnchor * 2];
};

/// Raw TensorRT detection output (GPU format).
struct alignas(float) RawDetection
{
  float bbox[4];  // center_x, center_y, w, h
  float conf;     // bbox_conf * cls_conf
  float class_id;
  float mask[32];
};

}  // namespace drone::detector
