// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_waypoint_nav.cpp
/// @brief Unit tests for waypoint navigation state management.

#include "drone/mission/waypoint_nav.hpp"

#include <gtest/gtest.h>

using drone::mission::WaypointState;

TEST(WaypointState, DefaultValues)
{
  WaypointState state;
  EXPECT_EQ(state.index, 0);
  EXPECT_FALSE(state.initialized);
}

TEST(WaypointState, ResetClearsAll)
{
  WaypointState state;
  state.index = 5;
  state.initialized = true;

  state.reset();

  EXPECT_EQ(state.index, 0);
  EXPECT_FALSE(state.initialized);
}

TEST(WaypointState, ResetTimerRestartsFromZero)
{
  WaypointState state;

  // Let some time pass (timer starts at construction)
  // After reset, elapsed should be very small
  state.reset();
  EXPECT_LT(state.timer.elapsed(), 0.1);
}
