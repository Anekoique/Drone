// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_velocity_controller.cpp
/// @brief Unit tests for the velocity controller and fuzzy-PID variant.

#include "drone/control/velocity_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::control::Limits;
using drone::control::VelocityController;

namespace
{

VelocityController::Config make_config()
{
  VelocityController::Config cfg;
  cfg.pid_x = {0.6f, 0.25f, 0.5f, 0, 0, 5, 0, 0, 0, 0, 0};
  cfg.pid_y = {0.6f, 0.25f, 0.5f, 0, 0, 5, 0, 0, 0, 0, 0};
  cfg.pid_z = {0.3f, 0.1f, 0.1f, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_yaw = {0.1f, 0.04f, 0.05f, 0, 0, 5, 0, 0, 0, 0, 0};
  cfg.limits.speed_max_xy = 2.0f;
  cfg.limits.speed_max_z = 1.0f;
  cfg.limits.speed_max_yaw = 0.3f;
  return cfg;
}

}  // namespace

TEST(VelocityController, ZeroOutputAtTarget)
{
  auto ctrl = VelocityController(make_config());
  Eigen::Vector3f pos(1, 2, 3);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();
  auto vel = ctrl.compute(pos, pos, 0.5f, 0.5f, 0.1f, vel_fb);
  EXPECT_NEAR(vel.x(), 0.0f, 1e-4f);
  EXPECT_NEAR(vel.y(), 0.0f, 1e-4f);
  EXPECT_NEAR(vel.z(), 0.0f, 1e-4f);
  EXPECT_NEAR(vel.w(), 0.0f, 1e-4f);
}

TEST(VelocityController, MovesTowardTarget)
{
  auto ctrl = VelocityController(make_config());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(5, 3, 2);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();
  auto vel = ctrl.compute(pos, target, 0.0f, 0.0f, 0.1f, vel_fb);

  // Velocity should be in the same direction as the error
  EXPECT_GT(vel.x(), 0.0f);
  EXPECT_GT(vel.y(), 0.0f);
  EXPECT_GT(vel.z(), 0.0f);
}

TEST(VelocityController, RespectsLimits)
{
  auto cfg = make_config();
  cfg.limits.speed_max_xy = 1.0f;
  cfg.limits.speed_max_z = 0.5f;
  auto ctrl = VelocityController(cfg);

  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(100, 100, 100);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();
  auto vel = ctrl.compute(pos, target, 0.0f, 0.0f, 0.1f, vel_fb);

  EXPECT_LE(std::abs(vel.x()), cfg.limits.speed_max_xy + 0.01f);
  EXPECT_LE(std::abs(vel.y()), cfg.limits.speed_max_xy + 0.01f);
  EXPECT_LE(std::abs(vel.z()), cfg.limits.speed_max_z + 0.01f);
}

TEST(VelocityController, NegativeDirectionWorks)
{
  auto ctrl = VelocityController(make_config());
  Eigen::Vector3f pos(5, 5, 5);
  Eigen::Vector3f target(0, 0, 0);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();
  auto vel = ctrl.compute(pos, target, 0.0f, 0.0f, 0.1f, vel_fb);

  EXPECT_LT(vel.x(), 0.0f);
  EXPECT_LT(vel.y(), 0.0f);
  EXPECT_LT(vel.z(), 0.0f);
}

TEST(VelocityController, YawWrapsCorrectly)
{
  auto ctrl = VelocityController(make_config());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();
  constexpr float kPi = static_cast<float>(M_PI);

  // Current yaw near +pi, target near -pi: shortest path is small positive
  auto vel = ctrl.compute(pos, pos, 0.9f * kPi, -0.9f * kPi, 0.1f, vel_fb);
  // The yaw rate should be small (wrapping through +/-pi)
  EXPECT_LT(std::abs(vel.w()), 0.3f);
}

TEST(VelocityController, ResetPidClearsIntegrator)
{
  auto ctrl = VelocityController(make_config());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(1, 0, 0);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();

  // Run several iterations to build up integrator
  for (int i = 0; i < 10; ++i) {
    ctrl.compute(pos, target, 0, 0, 0.1f, vel_fb);
  }

  ctrl.reset_pid();

  // After reset, the integrator should be cleared
  EXPECT_NEAR(ctrl.pid_x().get_integrator(), 0.0f, 1e-6f);
}
