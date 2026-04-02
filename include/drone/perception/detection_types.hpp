#pragma once

#include <Eigen/Core>

namespace drone
{

enum class TargetClass : int
{
  kCircle = 0,
  kStuffed = 1,
  kH = 2,
};

/// Raw detection output from the detector.
struct Detection
{
  float cx = 0;          // center x in pixels
  float cy = 0;          // center y in pixels
  float w = 0;           // width in pixels
  float h = 0;           // height in pixels
  float confidence = 0;  // bbox_conf * cls_conf
  int class_id = 0;      // 0=circle, 1=stuffed, 2=H
};

/// A detected target sample with world position and estimated diameter.
/// Used as clustering input — preserves diameter for shot-order ranking.
struct TargetSample
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double diameter = 0;
};

/// A cluster center representing a container target.
struct Target
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double diameter = 0;
  int cluster_id = 0;
};

}  // namespace drone
