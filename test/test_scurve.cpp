#include "drone/math/scurve.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::SCurve;

TEST(SCurve, CalculatePathProducesValidTimes)
{
  float Jm_out = 0, tj_out = 0, t2_out = 0, t4_out = 0, t6_out = 0;

  // Simple path: L=10m, Vm=2m/s, Am=1m/s^2, Jm=0.5m/s^3
  SCurve::calculate_path(
    0.5f,   // Sm (snap max)
    0.5f,   // Jm (jerk max)
    0.0f,   // V0 (initial velocity)
    1.0f,   // Am (accel max)
    2.0f,   // Vm (velocity max)
    10.0f,  // L (distance)
    Jm_out, tj_out, t2_out, t4_out, t6_out);

  // All time segments should be non-negative
  EXPECT_GE(tj_out, 0.0f);
  EXPECT_GE(t2_out, 0.0f);
  EXPECT_GE(t4_out, 0.0f);
  EXPECT_GE(t6_out, 0.0f);
}

TEST(SCurve, CalculatePathZeroDistance)
{
  float Jm_out = 0, tj_out = 0, t2_out = 0, t4_out = 0, t6_out = 0;

  SCurve::calculate_path(
    0.5f, 0.5f, 0.0f, 1.0f, 2.0f, 0.0f, Jm_out, tj_out, t2_out, t4_out, t6_out);

  // Zero distance: all times should be zero or near-zero
  EXPECT_NEAR(tj_out, 0.0f, 1e-3f);
  EXPECT_NEAR(t4_out, 0.0f, 1e-3f);
}

TEST(SCurve, ConstructAndInitFinishedByDefault)
{
  // A freshly initialized SCurve with no track is considered finished
  SCurve curve;
  curve.init();
  EXPECT_TRUE(curve.finished());
}
