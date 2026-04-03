#include "drone/math/basic_pid.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::BasicPID;

TEST(BasicPID, DefaultConstructZeroOutput)
{
  BasicPID pid;
  float out = pid.compute(0.0f, 0.0f, 0.1f);
  EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST(BasicPID, ProportionalTracksTarget)
{
  BasicPID pid;
  pid.kp_ = 1.0f;
  pid.output_limit_ = 10.0f;
  pid.integral_limit = 10.0f;

  float out = pid.compute(5.0f, 0.0f, 0.1f);
  // P-only: output = kp * error = 1.0 * 5.0 = 5.0
  EXPECT_NEAR(out, 5.0f, 0.1f);
}

TEST(BasicPID, OutputRespectsLimit)
{
  BasicPID pid;
  pid.kp_ = 10.0f;
  pid.output_limit_ = 2.0f;
  pid.integral_limit = 10.0f;

  float out = pid.compute(100.0f, 0.0f, 0.1f);
  EXPECT_LE(std::abs(out), 2.0f + 1e-6f);
}

TEST(BasicPID, IntegralAccumulates)
{
  BasicPID pid;
  pid.kp_ = 0.0f;
  pid.ki_ = 1.0f;
  pid.output_limit_ = 100.0f;
  pid.integral_limit = 100.0f;

  // Error = 1.0 for 10 iterations at dt=0.1 => integral = 1.0
  for (int i = 0; i < 10; ++i) {
    pid.compute(1.0f, 0.0f, 0.1f);
  }
  float out = pid.compute(1.0f, 0.0f, 0.1f);
  // After 11 iterations: integral = 1.0 * 0.1 * 11 = 1.1
  EXPECT_GT(out, 0.5f);
}

TEST(BasicPID, MypidSetsVelocities)
{
  BasicPID pid;
  pid.kp_ = 1.0f;
  pid.output_limit_ = 10.0f;
  pid.integral_limit = 10.0f;

  pid.Mypid(5.0f, 3.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.1f);

  // velocity_x = -compute(5, 0) = -5, velocity_y = -compute(3, 0) = -3, velocity_z = compute(2, 0) = 2
  EXPECT_LT(pid.velocity_x, 0.0f);
  EXPECT_LT(pid.velocity_y, 0.0f);
  EXPECT_GT(pid.velocity_z, 0.0f);
}
