// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_math_utils.cpp
/// @brief Unit tests for math utility functions.

#include "drone/math/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(MathUtilsTest, IsZero)
{
  EXPECT_TRUE(drone::is_zero(0.0f));
  EXPECT_FALSE(drone::is_zero(0.001f));
  EXPECT_FALSE(drone::is_zero(-1.0f));
}

TEST(MathUtilsTest, IsPositive)
{
  EXPECT_TRUE(drone::is_positive(0.0f));
  EXPECT_TRUE(drone::is_positive(1.0f));
  EXPECT_FALSE(drone::is_positive(-0.001f));
}

TEST(MathUtilsTest, ConstrainFloat)
{
  EXPECT_FLOAT_EQ(drone::constrain_float(5.0f, 10.0f, -10.0f), 5.0f);
  EXPECT_FLOAT_EQ(drone::constrain_float(15.0f, 10.0f, -10.0f), 10.0f);
  EXPECT_FLOAT_EQ(drone::constrain_float(-15.0f, 10.0f, -10.0f), -10.0f);
}

TEST(MathUtilsTest, SafeSqrt)
{
  EXPECT_FLOAT_EQ(drone::safe_sqrt(4.0f), 2.0f);
  EXPECT_FLOAT_EQ(drone::safe_sqrt(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(drone::safe_sqrt(-1.0f), 0.0f);  // negative → 0
}

TEST(MathUtilsTest, IsEqual)
{
  EXPECT_TRUE(drone::is_equal(1.0f, 1.0005f, 0.001f));
  EXPECT_FALSE(drone::is_equal(1.0f, 1.01f, 0.001f));
}
