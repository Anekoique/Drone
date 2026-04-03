// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <Eigen/Core>

namespace drone
{

/// Target class IDs matching the detector model output.
enum class TargetClass : int
{
  kCircle = 0,
  kStuffed = 1,
  kH = 2,
};

/// Raw detection output from the detector.
struct Detection
{
  float cx = 0;          ///< Center X in pixels
  float cy = 0;          ///< Center Y in pixels
  float w = 0;           ///< Bounding box width in pixels
  float h = 0;           ///< Bounding box height in pixels
  float confidence = 0;  ///< Combined confidence (bbox_conf * cls_conf)
  int class_id = 0;      ///< Target class (0=circle, 1=stuffed, 2=H)
};

/// A detected target sample with world position and estimated diameter.
/// Used as clustering input -- preserves diameter for shot-order ranking.
struct TargetSample
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  ///< World-frame position (ENU)
  double diameter = 0;                                 ///< Estimated real-world diameter (m)
};

/// A cluster center representing a container target.
struct Target
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();  ///< World-frame cluster center (ENU)
  double diameter = 0;                                 ///< Average diameter of cluster members (m)
  int cluster_id = 0;                                  ///< Cluster index
};

}  // namespace drone
