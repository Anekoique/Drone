#include "drone/perception/detector/postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <vector>

namespace drone::detector
{

cv::Rect get_rect(const cv::Mat & img, float bbox[4])
{
  float r_w = static_cast<float>(kInputW) / (img.cols * 1.0f);
  float r_h = static_cast<float>(kInputH) / (img.rows * 1.0f);
  float l, r, t, b;

  if (r_h > r_w) {
    l = bbox[0] - bbox[2] / 2.0f;
    r = bbox[0] + bbox[2] / 2.0f;
    t = bbox[1] - bbox[3] / 2.0f - (kInputH - r_w * img.rows) / 2.0f;
    b = bbox[1] + bbox[3] / 2.0f - (kInputH - r_w * img.rows) / 2.0f;
    l /= r_w;
    r /= r_w;
    t /= r_w;
    b /= r_w;
  } else {
    l = bbox[0] - bbox[2] / 2.0f - (kInputW - r_h * img.cols) / 2.0f;
    r = bbox[0] + bbox[2] / 2.0f - (kInputW - r_h * img.cols) / 2.0f;
    t = bbox[1] - bbox[3] / 2.0f;
    b = bbox[1] + bbox[3] / 2.0f;
    l /= r_h;
    r /= r_h;
    t /= r_h;
    b /= r_h;
  }

  l = std::max(0.0f, l);
  r = std::min(static_cast<float>(img.cols), r);
  t = std::max(0.0f, t);
  b = std::min(static_cast<float>(img.rows), b);
  return cv::Rect(
    static_cast<int>(std::round(l)), static_cast<int>(std::round(t)),
    static_cast<int>(std::round(r - l)), static_cast<int>(std::round(b - t)));
}

namespace
{

float iou(float lbox[4], float rbox[4])
{
  float inter[] = {
    std::max(lbox[0] - lbox[2] / 2.0f, rbox[0] - rbox[2] / 2.0f),
    std::min(lbox[0] + lbox[2] / 2.0f, rbox[0] + rbox[2] / 2.0f),
    std::max(lbox[1] - lbox[3] / 2.0f, rbox[1] - rbox[3] / 2.0f),
    std::min(lbox[1] + lbox[3] / 2.0f, rbox[1] + rbox[3] / 2.0f),
  };
  if (inter[2] > inter[3] || inter[0] > inter[1]) return 0.0f;
  float area = (inter[1] - inter[0]) * (inter[3] - inter[2]);
  return area / (lbox[2] * lbox[3] + rbox[2] * rbox[3] - area);
}

}  // namespace

void nms(std::vector<RawDetection> & res, float * output, float conf_thresh, float nms_thresh)
{
  int det_size = sizeof(RawDetection) / sizeof(float);
  std::map<float, std::vector<RawDetection>> class_map;

  for (int i = 0; i < static_cast<int>(output[0]) && i < kMaxNumOutputBbox; i++) {
    if (output[1 + det_size * i + 4] <= conf_thresh) continue;

    RawDetection det{};
    std::memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));

    // Clamp bbox to input bounds
    float left = std::max(det.bbox[0] - det.bbox[2] / 2.0f, 0.0f);
    float top = std::max(det.bbox[1] - det.bbox[3] / 2.0f, 0.0f);
    float right = std::min(det.bbox[0] + det.bbox[2] / 2.0f, kInputW - 1.0f);
    float bottom = std::min(det.bbox[1] + det.bbox[3] / 2.0f, kInputH - 1.0f);
    det.bbox[2] = right - left;
    det.bbox[3] = bottom - top;
    det.bbox[0] = left + det.bbox[2] / 2.0f;
    det.bbox[1] = top + det.bbox[3] / 2.0f;

    class_map[det.class_id].push_back(det);
  }

  for (auto & [cls, dets] : class_map) {
    std::sort(dets.begin(), dets.end(), [](const RawDetection & a, const RawDetection & b) {
      return a.conf > b.conf;
    });
    for (size_t m = 0; m < dets.size(); ++m) {
      res.push_back(dets[m]);
      for (size_t n = m + 1; n < dets.size(); ++n) {
        if (iou(dets[m].bbox, dets[n].bbox) > nms_thresh) {
          dets.erase(dets.begin() + static_cast<long>(n));
          --n;
        }
      }
    }
  }
}

}  // namespace drone::detector
