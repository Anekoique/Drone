// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file utils.cpp
/// @brief Math utility functions: vector equality, and kinematic limit computation
///        that resolves XY/Z speed/acceleration constraints along a 3D direction.

#include "drone/math/types.hpp"

#include <cmath>

namespace drone
{

bool is_equal(Eigen::Vector4f a, Eigen::Vector4f b, float tolerance)
{
  return std::abs(a.x() - b.x()) < tolerance && std::abs(a.y() - b.y()) < tolerance &&
         std::abs(a.z() - b.z()) < tolerance && std::abs(a.w() - b.w()) < tolerance;
}

/// Compute the maximum scalar speed/acceleration along a 3D direction vector,
/// constrained by separate horizontal (XY) and vertical (up/down) limits.
/// Projects the direction onto XY and Z components and picks the binding constraint.
float kinematic_limit(Eigen::Vector3f direction, float max_xy, float max_z_pos, float max_z_neg)
{
  if (
    is_zero(direction.squaredNorm()) || is_zero(max_xy) || is_zero(max_z_pos) ||
    is_zero(max_z_neg)) {
    return 0.0f;
  }

  max_xy = std::fabs(max_xy);
  max_z_pos = std::fabs(max_z_pos);
  max_z_neg = std::fabs(max_z_neg);

  direction.normalize();
  const float xy_length = std::sqrt(sq(direction.x()) + sq(direction.y()));

  if (is_zero(xy_length)) {
    return is_positive(direction.z()) ? max_z_pos : max_z_neg;
  }
  if (is_zero(direction.z())) {
    return max_xy;
  }

  const float slope = direction.z() / xy_length;
  if (is_positive(slope)) {
    if (std::fabs(slope) < max_z_pos / max_xy) {
      return max_xy / xy_length;
    }
    return std::fabs(max_z_pos / direction.z());
  }

  if (std::fabs(slope) < max_z_neg / max_xy) {
    return max_xy / xy_length;
  }
  return std::fabs(max_z_neg / direction.z());
}

}  // namespace drone
