#include "drone/control/cascade_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::control::CascadeController;

namespace
{

CascadeController::Config make_config(float dt_outer = 0.1f, float dt_inner = 1.0f)
{
  CascadeController::Config cfg;
  cfg.pid_px = {1.0f, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_py = {1.0f, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_pz = {1.0f, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_vx = {1.35f, 1.15f, 0.85f, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_vy = {1.35f, 1.15f, 0.85f, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_vz = {1.0f, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_yaw = {0.1f, 0.04f, 0.05f, 0, 0, 5, 0, 0, 0, 0, 0};
  cfg.limits.speed_max_xy = 2.0f;
  cfg.limits.speed_max_z = 1.0f;
  cfg.limits.accel_max_xy = 1.2f;
  cfg.limits.accel_max_z = 1.2f;
  cfg.limits.speed_max_yaw = 0.3f;
  cfg.dt_outer = dt_outer;
  cfg.dt_inner = dt_inner;
  return cfg;
}

}  // namespace

TEST(CascadeController, ZeroOutputAtTargetZeroVelocity)
{
  auto ctrl = CascadeController(make_config());
  Eigen::Vector3f pos(1, 2, 3);
  Eigen::Vector4f vel = Eigen::Vector4f::Zero();
  auto accel = ctrl.compute(pos, pos, vel, 0.5f, 0.5f);
  EXPECT_NEAR(accel.x(), 0.0f, 1e-4f);
  EXPECT_NEAR(accel.y(), 0.0f, 1e-4f);
  EXPECT_NEAR(accel.z(), 0.0f, 1e-4f);
  EXPECT_NEAR(accel.w(), 0.0f, 1e-4f);
}

TEST(CascadeController, ProducesOutputWithError)
{
  auto ctrl = CascadeController(make_config());
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(5, 3, 2);
  Eigen::Vector4f vel = Eigen::Vector4f::Zero();
  auto accel = ctrl.compute(pos, target, vel, 0.0f, 0.0f);

  // Should produce acceleration toward target
  EXPECT_GT(accel.x(), 0.0f);
  EXPECT_GT(accel.y(), 0.0f);
  EXPECT_GT(accel.z(), 0.0f);
}

TEST(CascadeController, RespectsAccelLimits)
{
  auto cfg = make_config();
  cfg.limits.accel_max_xy = 0.5f;
  cfg.limits.accel_max_z = 0.3f;
  auto ctrl = CascadeController(cfg);

  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(100, 100, 100);
  Eigen::Vector4f vel = Eigen::Vector4f::Zero();
  auto accel = ctrl.compute(pos, target, vel, 0.0f, 0.0f);

  EXPECT_LE(std::abs(accel.x()), cfg.limits.accel_max_xy + 0.01f);
  EXPECT_LE(std::abs(accel.y()), cfg.limits.accel_max_xy + 0.01f);
  EXPECT_LE(std::abs(accel.z()), cfg.limits.accel_max_z + 0.01f);
}

TEST(CascadeController, DifferentTimestepsProduceDifferentResults)
{
  // Use small errors so outputs don't saturate at accel limits
  auto cfg_legacy = make_config(0.1f, 1.0f);
  cfg_legacy.limits.accel_max_xy = 100.0f;
  auto cfg_equal = make_config(0.1f, 0.1f);
  cfg_equal.limits.accel_max_xy = 100.0f;

  auto ctrl_legacy = CascadeController(cfg_legacy);
  auto ctrl_equal = CascadeController(cfg_equal);

  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(0.1f, 0, 0);
  Eigen::Vector4f vel(0.05f, 0, 0, 0);

  // Run a few iterations to build up integrator difference
  Eigen::Vector4f accel_legacy, accel_equal;
  for (int i = 0; i < 5; ++i) {
    accel_legacy = ctrl_legacy.compute(pos, target, vel, 0.0f, 0.0f);
    accel_equal = ctrl_equal.compute(pos, target, vel, 0.0f, 0.0f);
  }

  // Different dt_inner should produce different inner-loop integrator accumulation
  EXPECT_NE(accel_legacy.x(), accel_equal.x());
}

TEST(CascadeController, ResetClearsIntegrator)
{
  auto cfg = make_config();
  cfg.limits.accel_max_xy = 100.0f;
  auto ctrl = CascadeController(cfg);
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(0.5f, 0, 0);
  Eigen::Vector4f vel = Eigen::Vector4f::Zero();

  // Build up integrator
  for (int i = 0; i < 10; ++i) {
    ctrl.compute(pos, target, vel, 0.0f, 0.0f);
  }

  auto before_reset = ctrl.compute(pos, target, vel, 0.0f, 0.0f);

  ctrl.reset_pid();

  // After reset, integrator should be zero so output differs from accumulated state
  auto after_reset = ctrl.compute(pos, target, vel, 0.0f, 0.0f);

  // The accumulated version should have a larger output due to integral buildup
  EXPECT_NE(before_reset.x(), after_reset.x());
}
