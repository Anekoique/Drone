#include "drone/math/pid.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::PID;

TEST(PIDTest, ConstructFromGains)
{
  PID pid(1.0f, 0.1f, 0.01f);
  float kp, ki, kd;
  pid.get_pid(kp, ki, kd);
  EXPECT_FLOAT_EQ(kp, 1.0f);
  EXPECT_FLOAT_EQ(ki, 0.1f);
  EXPECT_FLOAT_EQ(kd, 0.01f);
}

TEST(PIDTest, ConstructFromDefaults)
{
  PID::Defaults d;
  d.p = 2.0f;
  d.i = 0.5f;
  d.d = 0.1f;
  PID pid(d);
  float kp, ki, kd;
  pid.get_pid(kp, ki, kd);
  EXPECT_FLOAT_EQ(kp, 2.0f);
}

TEST(PIDTest, UpdateAllConverges)
{
  PID pid("test", 0.5f, 0.1f, 0.05f);
  float value = 0.0f;
  float target = 10.0f;

  for (int i = 0; i < 200; i++) {
    float output = pid.update_all(value, target, 0.05f, 20.0f);
    value += output * 0.05f;
  }

  EXPECT_NEAR(value, target, 2.0f);
}

TEST(PIDTest, OutputRespectLimit)
{
  PID pid(10.0f, 0.0f, 0.0f);  // high P, no I/D
  float output = pid.update_all(0.0f, 100.0f, 0.05f, 5.0f);
  EXPECT_LE(std::abs(output), 5.0f);
}

TEST(PIDTest, ResetAll)
{
  PID pid(1.0f, 0.1f, 0.01f);
  pid.update_all(0.0f, 10.0f, 0.05f, 20.0f);
  pid.reset_all();
  EXPECT_FLOAT_EQ(pid.get_integrator(), 0.0f);
  EXPECT_FLOAT_EQ(pid._error, 0.0f);
}

TEST(PIDTest, SetAndGetIntegrator)
{
  PID pid(1.0f, 0.1f, 0.01f, 0, 0, 5.0f);
  pid.set_integrator(3.0f);
  EXPECT_FLOAT_EQ(pid.get_integrator(), 3.0f);

  // Should clamp to imax
  pid.set_integrator(100.0f);
  EXPECT_FLOAT_EQ(pid.get_integrator(), 5.0f);
}
