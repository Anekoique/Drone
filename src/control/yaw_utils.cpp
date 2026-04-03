// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file yaw_utils.cpp
/// @brief Yaw angle normalization and shortest-path error computation.

#include "drone/control/yaw_utils.hpp"

#include <cmath>

namespace drone::control
{

float normalize_yaw(float yaw)
{
  while (yaw > static_cast<float>(M_PI)) {
    yaw -= 2.0f * static_cast<float>(M_PI);
  }
  while (yaw < -static_cast<float>(M_PI)) {
    yaw += 2.0f * static_cast<float>(M_PI);
  }
  return yaw;
}

float yaw_error(float current, float target)
{
  return normalize_yaw(target - current);
}

}  // namespace drone::control
