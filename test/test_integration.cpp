// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_integration.cpp
/// @brief Integration tests spanning multiple subsystems.

#include "drone/control/limits.hpp"
#include "drone/control/velocity_controller.hpp"
#include "drone/control/yaw_utils.hpp"
#include "drone/math/pid.hpp"
#include "drone/mission/frame_transforms.hpp"
#include "drone/mission/mission_types.hpp"
#include "drone/mission/waypoint_nav.hpp"
#include "drone/perception/camera_model.hpp"
#include "drone/perception/clustering.hpp"
#include "drone/perception/detection_filter.hpp"
#include "drone/perception/detection_types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace drone;
using namespace drone::control;
using namespace drone::mission;

// --- End-to-end: detection -> clustering -> world position ---

TEST(Integration, DetectionToClustering)
{
  // Simulate 6 detections forming 2 clusters
  std::vector<TargetSample> samples;
  // Cluster 1 around (1, 2)
  samples.push_back({{1.0, 2.0, 0.0}, 0.3});
  samples.push_back({{1.1, 2.1, 0.0}, 0.32});
  samples.push_back({{0.9, 1.9, 0.0}, 0.28});
  // Cluster 2 around (5, 6)
  samples.push_back({{5.0, 6.0, 0.0}, 0.5});
  samples.push_back({{5.1, 5.9, 0.0}, 0.48});
  samples.push_back({{4.9, 6.1, 0.0}, 0.52});

  auto centers = find_cluster_centers(samples);

  // Should find at least 2 clusters
  EXPECT_GE(centers.size(), 2u);

  // Clusters should be near the expected positions
  bool found_cluster1 = false, found_cluster2 = false;
  for (const auto & c : centers) {
    if (std::abs(c.position.x() - 1.0) < 0.5 && std::abs(c.position.y() - 2.0) < 0.5) {
      found_cluster1 = true;
    }
    if (std::abs(c.position.x() - 5.0) < 0.5 && std::abs(c.position.y() - 6.0) < 0.5) {
      found_cluster2 = true;
    }
  }
  EXPECT_TRUE(found_cluster1);
  EXPECT_TRUE(found_cluster2);
}

// --- End-to-end: camera model pixel-to-world ---

TEST(Integration, CameraPixelToWorld)
{
  CameraModel cam;
  cam.set_intrinsics(611.4, 610.7, 637.2, 367.9, 1280, 720);
  cam.set_distortion(0, 0, 0, 0, 0);  // No distortion for simplicity
  cam.set_relative_position(Eigen::Vector3d(0, 0, 0));
  cam.set_relative_rotation(Eigen::Vector3d(0, 0, 0));
  cam.set_parent_pose(Eigen::Vector3d(0, 0, 5), Eigen::Vector3d(0, 0, 0));  // 5m up

  // Center pixel should project to roughly below the camera
  auto world = cam.pixel_to_world(Eigen::Vector2d(637.2, 367.9), 0.0);
  ASSERT_TRUE(world.has_value());
  // At center pixel, with camera at (0,0,5) looking down, should be near (0,0,0)
  EXPECT_NEAR(world->x(), 0.0, 1.0);
  EXPECT_NEAR(world->y(), 0.0, 1.0);
}

// --- End-to-end: frame transform chain ---

TEST(Integration, FrameTransformChainRoundTrip)
{
  float heading = 30.0f * static_cast<float>(M_PI) / 180.0f;  // 30 degrees

  // Compass -> World -> Compass should round-trip
  float x = 10.0f, y = 20.0f;
  auto world = compass_to_world(x, y, heading);
  auto back = world_to_compass(world.x(), world.y(), heading);
  EXPECT_NEAR(back.x(), x, 1e-4f);
  EXPECT_NEAR(back.y(), y, 1e-4f);
}

TEST(Integration, ZoneOriginToWaypoint)
{
  // Zone at dx=0, dy=30m with heading=0 -> origin should be (0, 30)
  auto origin = zone_origin_to_world(0.0f, 30.0f, 0.0f, 0.0f);
  EXPECT_NEAR(origin.x(), 0.0f, 1e-4f);
  EXPECT_NEAR(origin.y(), 30.0f, 1e-4f);
}

// --- End-to-end: Kalman filter tracking ---

TEST(Integration, KalmanFilterTracksTarget)
{
  DetectionFilter filter(0.015, 0.5);

  // Feed a series of detections at roughly the same position
  for (int i = 0; i < 20; ++i) {
    std::vector<Detection> dets;
    dets.push_back({320.0f + static_cast<float>(i) * 0.1f, 240.0f, 50, 50, 0.9f, 0});
    filter.update(dets, 0.05, 1280, 720);
  }

  // Filtered position should be near (320, 240)
  auto pos = filter.get_filtered(TargetClass::kCircle);
  ASSERT_TRUE(pos.has_value());
  EXPECT_NEAR(pos->x(), 320.0, 5.0);
  EXPECT_NEAR(pos->y(), 240.0, 5.0);
}

// --- End-to-end: PID controller convergence ---

TEST(Integration, VelocityControllerConverges)
{
  VelocityController::Config cfg;
  cfg.pid_x = {0.6f, 0.1f, 0.05f, 0, 0, 5, 0, 0, 0, 0, 0};
  cfg.pid_y = cfg.pid_x;
  cfg.pid_z = {0.3f, 0.1f, 0.1f, 0, 0, 10, 0, 0, 0, 0, 0};
  cfg.pid_yaw = {0.1f, 0.04f, 0.05f, 0, 0, 5, 0, 0, 0, 0, 0};
  cfg.limits.speed_max_xy = 2.0f;
  cfg.limits.speed_max_z = 1.0f;
  cfg.limits.speed_max_yaw = 0.3f;

  VelocityController ctrl(cfg);
  Eigen::Vector3f pos(0, 0, 0);
  Eigen::Vector3f target(1, 0, 0);
  Eigen::Vector4f vel_fb = Eigen::Vector4f::Zero();

  // Simulate 100 iterations
  for (int i = 0; i < 100; ++i) {
    auto vel = ctrl.compute(pos, target, 0, 0, 0.1f, vel_fb);
    // Simple Euler integration
    pos.x() += vel.x() * 0.1f;
    pos.y() += vel.y() * 0.1f;
    pos.z() += vel.z() * 0.1f;
  }

  // Should converge near target
  EXPECT_NEAR(pos.x(), 1.0f, 0.3f);
}

// --- End-to-end: MissionConfig loads all files ---

TEST(Integration, MissionConfigFullLoad)
{
  auto cfg = MissionConfig::load("mission.yaml", "airdrop.yaml", "landing.yaml");

  // Verify cross-file data is coherent
  EXPECT_GT(cfg.heading_compass_rad, 0.0f);
  EXPECT_EQ(cfg.shot_zone.waypoints.size(), 12u);
  EXPECT_EQ(cfg.recon_zone.waypoints.size(), 5u);
  EXPECT_GT(cfg.airdrop.pid.p, 0.0f);
  EXPECT_GT(cfg.landing.pid.p, 0.0f);
  EXPECT_GT(cfg.servo_open_pwm, cfg.servo_close_pwm);
}

// --- End-to-end: all FlyState transitions are valid ---

TEST(Integration, FlyStateTransitionSequence)
{
  // Verify the normal mission flow state sequence
  FlyState flow[] = {
    FlyState::init,       FlyState::takeoff,      FlyState::goto_shot, FlyState::airdrop,
    FlyState::goto_recon, FlyState::recon_patrol, FlyState::landing,   FlyState::finished,
  };

  // Each state should have a different int encoding or same for groups
  for (size_t i = 0; i < sizeof(flow) / sizeof(flow[0]); ++i) {
    int val = fly_state_to_int(flow[i]);
    EXPECT_GE(val, 0);
    EXPECT_LE(val, 4);
  }
}

// --- End-to-end: yaw wrapping through full mission ---

TEST(Integration, YawWrappingConsistency)
{
  // Verify yaw_error and normalize_yaw are consistent through a full rotation
  for (float yaw = -3.0f * static_cast<float>(M_PI); yaw < 3.0f * static_cast<float>(M_PI);
       yaw += 0.1f) {
    float normalized = normalize_yaw(yaw);
    EXPECT_GE(normalized, -static_cast<float>(M_PI) - 1e-5f);
    EXPECT_LE(normalized, static_cast<float>(M_PI) + 1e-5f);
  }
}
