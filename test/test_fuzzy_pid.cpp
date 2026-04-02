#include "drone/control/fuzzy_pid.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::FuzzyPID;

TEST(FuzzyPIDTest, DefaultConstruction)
{
  FuzzyPID fpid;
  // Should not crash — default rule tables initialized
  float kp = 1.0f, ki = 0.5f, kd = 0.1f;
  fpid.fuzzy_pid_control(0.0f, 10.0f, 0, kp, ki, kd);
  // Gains should be modified by fuzzy controller
  EXPECT_TRUE(std::isfinite(kp));
  EXPECT_TRUE(std::isfinite(ki));
  EXPECT_TRUE(std::isfinite(kd));
}

TEST(FuzzyPIDTest, ZeroErrorProducesSmallDelta)
{
  FuzzyPID fpid;
  float kp = 1.0f, ki = 0.5f, kd = 0.1f;
  fpid.fuzzy_pid_control(10.0f, 10.0f, 0, kp, ki, kd);  // error = 0
  // With zero error, gains should change minimally
  EXPECT_NEAR(kp, 1.0f, 0.5f);
}
