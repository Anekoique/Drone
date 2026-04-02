#include "drone/utils/rotate.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(RotateTest, ZeroAngle)
{
  auto [rx, ry] = drone::rotate_2d(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(rx, 1.0f, 1e-6f);
  EXPECT_NEAR(ry, 0.0f, 1e-6f);
}

TEST(RotateTest, Rotate90Degrees)
{
  auto [rx, ry] = drone::rotate_2d(1.0f, 0.0f, static_cast<float>(M_PI_2));
  EXPECT_NEAR(rx, 0.0f, 1e-5f);
  EXPECT_NEAR(ry, 1.0f, 1e-5f);
}

TEST(RotateTest, Rotate180Degrees)
{
  auto [rx, ry] = drone::rotate_2d(1.0f, 0.0f, static_cast<float>(M_PI));
  EXPECT_NEAR(rx, -1.0f, 1e-5f);
  EXPECT_NEAR(ry, 0.0f, 1e-5f);
}

TEST(RotateTest, DoublePrecision)
{
  auto [rx, ry] = drone::rotate_2d(1.0, 0.0, M_PI_2);
  EXPECT_NEAR(rx, 0.0, 1e-10);
  EXPECT_NEAR(ry, 1.0, 1e-10);
}

TEST(RotateTest, Immutable)
{
  float x = 1.0f, y = 0.0f;
  auto [rx, ry] = drone::rotate_2d(x, y, static_cast<float>(M_PI_2));
  // Original values unchanged
  EXPECT_FLOAT_EQ(x, 1.0f);
  EXPECT_FLOAT_EQ(y, 0.0f);
  (void)rx;
  (void)ry;
}
