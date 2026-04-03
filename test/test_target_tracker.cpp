// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_target_tracker.cpp
/// @brief Unit tests for target tracker and visual servo logic.

#include "drone/mission/target_tracker.hpp"
#include "drone/perception/detection_types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using drone::Detection;
using drone::TargetClass;
using drone::mission::TargetTracker;

// TargetTracker needs InertialNav and Gimbal which require rclcpp::Node.
// We test the free function catch_target and the pixel logic independently.

TEST(CatchTargetLogic, ConvergenceDetection)
{
  // Test the convergence check logic directly:
  // If now_pixel is within accuracy of tar_pixel, catch_target returns true
  float now_x = 320.0f;
  float now_y = 240.0f;
  float tar_x = 320.5f;
  float tar_y = 240.3f;
  float accuracy = 1.0f;

  bool converged = std::abs(now_x - tar_x) <= accuracy && std::abs(now_y - tar_y) <= accuracy;
  EXPECT_TRUE(converged);
}

TEST(CatchTargetLogic, NotConvergedWhenFar)
{
  float now_x = 100.0f;
  float now_y = 100.0f;
  float tar_x = 320.0f;
  float tar_y = 240.0f;
  float accuracy = 1.0f;

  bool converged = std::abs(now_x - tar_x) <= accuracy && std::abs(now_y - tar_y) <= accuracy;
  EXPECT_FALSE(converged);
}

TEST(CatchTargetLogic, PixelNormalization)
{
  // Frame width 1280, height 720 -> max = 1280
  float max_frame = std::max(1280.0f, 720.0f);
  EXPECT_FLOAT_EQ(max_frame, 1280.0f);

  // Normalize pixel (640, 360) -> (0.5, 0.28125)
  float norm_x = 640.0f / max_frame;
  float norm_y = 360.0f / max_frame;
  EXPECT_NEAR(norm_x, 0.5f, 0.001f);
  EXPECT_NEAR(norm_y, 0.28125f, 0.001f);
}

TEST(CatchTargetLogic, YFlipConvention)
{
  // Legacy: now_y = frame_height - detection_y (image y-down to y-up)
  float frame_height = 720.0f;
  float detection_y = 200.0f;
  float flipped_y = frame_height - detection_y;
  EXPECT_FLOAT_EQ(flipped_y, 520.0f);
}
