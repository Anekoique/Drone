// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_timer.cpp
/// @brief Unit tests for the Timer utility.

#include "drone/utils/timer.hpp"

#include <gtest/gtest.h>

#include <thread>

using drone::Timer;

TEST(TimerTest, ElapsedStartsPositive)
{
  Timer t;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GT(t.elapsed(), 0.005);
}

TEST(TimerTest, ResetResetsElapsed)
{
  Timer t;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  t.reset();
  EXPECT_LT(t.elapsed(), 0.01);
}

TEST(TimerTest, InvalidateReturnsZero)
{
  Timer t;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  t.invalidate();
  EXPECT_DOUBLE_EQ(t.elapsed(), 0.0);
}

TEST(TimerTest, SetElapsed)
{
  Timer t;
  t.set_elapsed(5.0);
  EXPECT_NEAR(t.elapsed(), 5.0, 0.1);
}

TEST(TimerTest, TimepointUninitializedReturnsZero)
{
  Timer t;
  EXPECT_DOUBLE_EQ(t.timepoint_elapsed(), 0.0);
}

TEST(TimerTest, TimepointTracks)
{
  Timer t;
  t.set_timepoint();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_GT(t.timepoint_elapsed(), 0.005);
}
