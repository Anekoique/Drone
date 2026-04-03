// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

namespace drone::control
{

/// Normalize angle to [-pi, pi].
/// @param yaw Angle in radians.
/// @return Wrapped angle in [-pi, pi].
float normalize_yaw(float yaw);

/// Shortest angular difference from current to target.
/// @param current Current yaw in radians.
/// @param target Target yaw in radians.
/// @return Signed error in [-pi, pi].
float yaw_error(float current, float target);

}  // namespace drone::control
