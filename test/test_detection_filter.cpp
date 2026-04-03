// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_detection_filter.cpp
/// @brief Unit tests for per-class Kalman detection filtering.

#include "drone/perception/detection_filter.hpp"

#include <gtest/gtest.h>

using drone::Detection;
using drone::DetectionFilter;
using drone::TargetClass;

TEST(DetectionFilterTest, NoDetectionsReturnsNullopt)
{
  DetectionFilter filter;
  filter.update({}, 0.05, 640, 480);
  EXPECT_FALSE(filter.get_filtered(TargetClass::kCircle).has_value());
  EXPECT_FALSE(filter.has_target(TargetClass::kCircle));
}

TEST(DetectionFilterTest, SingleDetectionInitializesFilter)
{
  DetectionFilter filter;
  Detection det{.cx = 320, .cy = 240, .w = 50, .h = 50, .confidence = 0.9f, .class_id = 0};
  filter.update({det}, 0.05, 640, 480);

  EXPECT_TRUE(filter.has_target(TargetClass::kCircle));
  auto pos = filter.get_filtered(TargetClass::kCircle);
  ASSERT_TRUE(pos.has_value());
  EXPECT_NEAR(pos->x(), 320, 1.0);
  EXPECT_NEAR(pos->y(), 240, 1.0);
}

TEST(DetectionFilterTest, FilterConvergesToStablePosition)
{
  DetectionFilter filter(0.01, 0.5);

  // Feed noisy detections around (300, 200)
  for (int i = 0; i < 20; i++) {
    float noise_x = (i % 3 == 0) ? 5.0f : -3.0f;
    float noise_y = (i % 2 == 0) ? 4.0f : -2.0f;
    Detection det{
      .cx = 300.0f + noise_x,
      .cy = 200.0f + noise_y,
      .w = 50,
      .h = 50,
      .confidence = 0.9f,
      .class_id = 0};
    filter.update({det}, 0.05, 640, 480);
  }

  auto pos = filter.get_filtered(TargetClass::kCircle);
  ASSERT_TRUE(pos.has_value());
  EXPECT_NEAR(pos->x(), 300, 10.0);
  EXPECT_NEAR(pos->y(), 200, 10.0);
}

TEST(DetectionFilterTest, MultiClassTracking)
{
  DetectionFilter filter;
  Detection circle{.cx = 100, .cy = 100, .w = 30, .h = 30, .confidence = 0.9f, .class_id = 0};
  Detection h_mark{.cx = 500, .cy = 400, .w = 60, .h = 60, .confidence = 0.8f, .class_id = 2};
  filter.update({circle, h_mark}, 0.05, 640, 480);

  EXPECT_TRUE(filter.has_target(TargetClass::kCircle));
  EXPECT_TRUE(filter.has_target(TargetClass::kH));
  EXPECT_FALSE(filter.has_target(TargetClass::kStuffed));
}

TEST(DetectionFilterTest, RawDetectionsSortedByCenter)
{
  DetectionFilter filter;
  Detection far{.cx = 10, .cy = 10, .w = 30, .h = 30, .confidence = 0.9f, .class_id = 0};
  Detection close{.cx = 320, .cy = 240, .w = 30, .h = 30, .confidence = 0.8f, .class_id = 0};
  filter.update({far, close}, 0.05, 640, 480);

  const auto & raw = filter.get_raw(TargetClass::kCircle);
  ASSERT_EQ(raw.size(), 2);
  // Closer to center should be first
  EXPECT_NEAR(raw[0].cx, 320, 1.0);
}
