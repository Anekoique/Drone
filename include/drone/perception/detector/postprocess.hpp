#pragma once

#include "drone/perception/detector/types.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace drone::detector
{

cv::Rect get_rect(const cv::Mat & img, float bbox[4]);

void nms(
  std::vector<RawDetection> & res, float * output, float conf_thresh, float nms_thresh = 0.5f);

}  // namespace drone::detector
