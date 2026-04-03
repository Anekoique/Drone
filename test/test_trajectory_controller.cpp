#include "drone/control/trajectory_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::control::Limits;
using drone::control::TrajectoryController;

namespace
{

Limits make_limits()
{
  Limits lim;
  lim.speed_max_xy = 2.0f;
  lim.speed_max_z = 1.0f;
  lim.accel_max_xy = 1.2f;
  lim.accel_max_z = 1.2f;
  return lim;
}

}  // namespace

TEST(TrajectoryController, SetpointWorldReturnsTarget)
{
  auto ctrl = TrajectoryController(make_limits());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector4f target(5, 3, 2, 0.5f);

  auto [waypoint, done] = ctrl.setpoint_world(pos, target);
  EXPECT_FALSE(done);
  EXPECT_NEAR(waypoint.x(), 5.0f, 1e-4f);
  EXPECT_NEAR(waypoint.y(), 3.0f, 1e-4f);
  EXPECT_NEAR(waypoint.z(), 2.0f, 1e-4f);
}

TEST(TrajectoryController, SetpointWorldDetectsArrival)
{
  auto ctrl = TrajectoryController(make_limits());
  Eigen::Vector3f pos(5, 3, 2);
  Eigen::Vector4f target(5, 3, 2, 0.5f);

  auto [waypoint, done] = ctrl.setpoint_world(pos, target, 0.1f);
  EXPECT_TRUE(done);
}

TEST(TrajectoryController, SetpointWorldDetectsTargetChange)
{
  auto ctrl = TrajectoryController(make_limits());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector4f target1(5, 3, 2, 0);
  Eigen::Vector4f target2(10, 6, 4, 0);

  auto [wp1, done1] = ctrl.setpoint_world(pos, target1);
  EXPECT_NEAR(wp1.x(), 5.0f, 1e-4f);

  auto [wp2, done2] = ctrl.setpoint_world(pos, target2);
  EXPECT_NEAR(wp2.x(), 10.0f, 1e-4f);
}

TEST(TrajectoryController, SetpointRelativeConvertsToAbsolute)
{
  auto ctrl = TrajectoryController(make_limits());
  Eigen::Vector3f pos(10, 20, 5);
  Eigen::Vector4f offset(2, 3, 1, 0);

  auto [waypoint, done] = ctrl.setpoint_relative(pos, offset);
  EXPECT_FALSE(done);
  EXPECT_NEAR(waypoint.x(), 12.0f, 1e-4f);
  EXPECT_NEAR(waypoint.y(), 23.0f, 1e-4f);
  EXPECT_NEAR(waypoint.z(), 6.0f, 1e-4f);
}

TEST(TrajectoryController, CircleProducesCircularMotion)
{
  auto ctrl = TrajectoryController(make_limits());
  constexpr float a = 5.0f;
  constexpr float b = 3.0f;
  constexpr float height = 10.0f;
  constexpr float angular_vel = 0.1f;
  constexpr float yaw = 0.0f;

  std::vector<Eigen::Vector4f> points;
  for (int i = 0; i < 20; ++i) {
    auto [wp, done] = ctrl.circle(a, b, height, angular_vel, yaw);
    points.push_back(wp);
    EXPECT_FALSE(done);  // Legacy never returns done
  }

  // All points should be at the specified height
  for (const auto & p : points) {
    EXPECT_NEAR(p.z(), height, 1e-4f);
  }

  // Points should trace an ellipse: (x/a)^2 + (y/b)^2 ~= 1
  for (const auto & p : points) {
    float ellipse = (p.x() * p.x()) / (a * a) + (p.y() * p.y()) / (b * b);
    EXPECT_NEAR(ellipse, 1.0f, 0.1f);
  }
}

TEST(TrajectoryController, ResetCircleStartsFromZeroAngle)
{
  auto ctrl = TrajectoryController(make_limits());
  float angular_vel = 0.5f;

  // Run some iterations
  for (int i = 0; i < 10; ++i) {
    ctrl.circle(5.0f, 5.0f, 10.0f, angular_vel, 0.0f);
  }

  // Reset and start again
  ctrl.reset_circle();
  auto [wp1, d1] = ctrl.circle(5.0f, 5.0f, 10.0f, angular_vel, 0.0f);

  // Fresh controller for comparison
  auto ctrl2 = TrajectoryController(make_limits());
  auto [wp2, d2] = ctrl2.circle(5.0f, 5.0f, 10.0f, angular_vel, 0.0f);

  EXPECT_NEAR(wp1.x(), wp2.x(), 1e-4f);
  EXPECT_NEAR(wp1.y(), wp2.y(), 1e-4f);
}

TEST(TrajectoryController, ResetAllClearsAllState)
{
  auto ctrl = TrajectoryController(make_limits());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector4f target(5, 3, 2, 0);

  ctrl.setpoint_world(pos, target);
  ctrl.circle(5, 5, 10, 0.1f, 0);

  ctrl.reset_all();

  // After reset, setpoint should reinitialize
  auto [wp, done] = ctrl.setpoint_world(pos, target);
  EXPECT_NEAR(wp.x(), 5.0f, 1e-4f);
}
