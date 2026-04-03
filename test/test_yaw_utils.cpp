#include "drone/control/yaw_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::control::normalize_yaw;
using drone::control::yaw_error;

constexpr float kPi = static_cast<float>(M_PI);
constexpr float kTol = 1e-5f;

// --- normalize_yaw ---

TEST(YawUtils, NormalizeZero)
{
  EXPECT_NEAR(normalize_yaw(0.0f), 0.0f, kTol);
}

TEST(YawUtils, NormalizePi)
{
  EXPECT_NEAR(normalize_yaw(kPi), kPi, kTol);
}

TEST(YawUtils, NormalizeNegativePi)
{
  EXPECT_NEAR(normalize_yaw(-kPi), -kPi, kTol);
}

TEST(YawUtils, NormalizeOverPi)
{
  // 1.5 * pi should wrap to -0.5 * pi
  EXPECT_NEAR(normalize_yaw(1.5f * kPi), -0.5f * kPi, kTol);
}

TEST(YawUtils, NormalizeUnderNegPi)
{
  // -1.5 * pi should wrap to 0.5 * pi
  EXPECT_NEAR(normalize_yaw(-1.5f * kPi), 0.5f * kPi, kTol);
}

TEST(YawUtils, NormalizeTwoPi)
{
  EXPECT_NEAR(normalize_yaw(2.0f * kPi), 0.0f, kTol);
}

TEST(YawUtils, NormalizeNegTwoPi)
{
  EXPECT_NEAR(normalize_yaw(-2.0f * kPi), 0.0f, kTol);
}

TEST(YawUtils, NormalizeLargePositive)
{
  // 3 * pi should normalize to pi
  EXPECT_NEAR(normalize_yaw(3.0f * kPi), kPi, kTol);
}

// --- yaw_error ---

TEST(YawUtils, ErrorSameAngle)
{
  EXPECT_NEAR(yaw_error(0.0f, 0.0f), 0.0f, kTol);
}

TEST(YawUtils, ErrorSmallPositive)
{
  EXPECT_NEAR(yaw_error(0.0f, 0.5f), 0.5f, kTol);
}

TEST(YawUtils, ErrorSmallNegative)
{
  EXPECT_NEAR(yaw_error(0.5f, 0.0f), -0.5f, kTol);
}

TEST(YawUtils, ErrorWrapsPositive)
{
  // From 170 deg to -170 deg: shortest path is +20 deg (through +180)
  float from = 170.0f / 180.0f * kPi;
  float to = -170.0f / 180.0f * kPi;
  float err = yaw_error(from, to);
  EXPECT_NEAR(err, 20.0f / 180.0f * kPi, kTol);
}

TEST(YawUtils, ErrorWrapsNegative)
{
  // From -170 deg to 170 deg: shortest path is -20 deg (through -180)
  float from = -170.0f / 180.0f * kPi;
  float to = 170.0f / 180.0f * kPi;
  float err = yaw_error(from, to);
  EXPECT_NEAR(err, -20.0f / 180.0f * kPi, kTol);
}

TEST(YawUtils, ErrorOpposite)
{
  // 180 degree difference
  float err = yaw_error(0.0f, kPi);
  EXPECT_NEAR(std::abs(err), kPi, kTol);
}
