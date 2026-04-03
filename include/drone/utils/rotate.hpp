// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cmath>
#include <concepts>
#include <utility>

namespace drone
{

/// Rotate a 2D point (x, y) clockwise by `angle` radians around the origin.
template <std::floating_point T>
[[nodiscard]] std::pair<T, T> rotate_2d(T x, T y, T angle)
{
  const T cs = std::cos(angle);
  const T sn = std::sin(angle);
  return {x * cs - y * sn, x * sn + y * cs};
}

}  // namespace drone
