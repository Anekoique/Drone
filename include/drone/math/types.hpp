// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/math/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace drone
{

/// Default floating-point type alias used across math utilities.
using ftype = float;

/// Square a scalar value.
template <typename T>
ftype sq(T val)
{
  auto v = static_cast<ftype>(val);
  return v * v;
}

/// Euclidean norm of a 3D vector.
template <typename T>
T norm(const T & x, const T & y, const T & z)
{
  return std::sqrt(x * x + y * y + z * z);
}

/// Euclidean norm of a 2D vector.
template <typename T>
T norm(const T & x, const T & y)
{
  return std::sqrt(x * x + y * y);
}

/// Rotate vector by angle in radians in XY plane, leaving Z untouched.
template <typename T>
void rotate_xy(T & x, T & y, float rotation_rad)
{
  const T cs = std::cos(rotation_rad);
  const T sn = std::sin(rotation_rad);
  T rx = x * cs - y * sn;
  T ry = x * sn + y * cs;
  x = rx;
  y = ry;
}

/// Check if two 4D vectors are approximately equal within tolerance.
bool is_equal(Eigen::Vector4f a, Eigen::Vector4f b, float tolerance = 0.001f);

/// Compute the maximum speed along a direction given separate XY and Z limits.
/// @param direction Unit direction vector.
/// @param max_xy Maximum horizontal speed.
/// @param max_z_pos Maximum upward speed.
/// @param max_z_neg Maximum downward speed.
/// @return Speed limit along direction.
float kinematic_limit(Eigen::Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);

}  // namespace drone
