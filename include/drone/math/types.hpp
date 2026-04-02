#pragma once

#include "drone/math/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace drone
{

using ftype = float;

template <typename T>
ftype sq(T val)
{
  auto v = static_cast<ftype>(val);
  return v * v;
}

template <typename T>
T norm(const T & x, const T & y, const T & z)
{
  return std::sqrt(x * x + y * y + z * z);
}

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

bool is_equal(Eigen::Vector4f a, Eigen::Vector4f b, float tolerance = 0.001f);

float kinematic_limit(Eigen::Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);

}  // namespace drone
