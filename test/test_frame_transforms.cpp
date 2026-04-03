#include "drone/mission/frame_transforms.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::mission::compass_to_world;
using drone::mission::local_to_world;
using drone::mission::rotate_xy;
using drone::mission::world_to_compass;
using drone::mission::world_to_local;
using drone::mission::world_to_start;
using drone::mission::zone_origin_to_world;

constexpr float kPi = static_cast<float>(M_PI);
constexpr float kTol = 1e-4f;

TEST(FrameTransforms, RotateXyZeroAngle)
{
  auto r = rotate_xy(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(r.x(), 1.0f, kTol);
  EXPECT_NEAR(r.y(), 0.0f, kTol);
}

TEST(FrameTransforms, RotateXy90Degrees)
{
  auto r = rotate_xy(1.0f, 0.0f, kPi / 2.0f);
  EXPECT_NEAR(r.x(), 0.0f, kTol);
  EXPECT_NEAR(r.y(), 1.0f, kTol);
}

TEST(FrameTransforms, RotateXy180Degrees)
{
  auto r = rotate_xy(1.0f, 0.0f, kPi);
  EXPECT_NEAR(r.x(), -1.0f, kTol);
  EXPECT_NEAR(r.y(), 0.0f, kTol);
}

TEST(FrameTransforms, RotateXy270Degrees)
{
  auto r = rotate_xy(1.0f, 0.0f, 3.0f * kPi / 2.0f);
  EXPECT_NEAR(r.x(), 0.0f, kTol);
  EXPECT_NEAR(r.y(), -1.0f, kTol);
}

TEST(FrameTransforms, CompassWorldRoundTrip)
{
  float heading = 1.2f;
  auto w = compass_to_world(3.0f, 4.0f, heading);
  auto c = world_to_compass(w.x(), w.y(), heading);
  EXPECT_NEAR(c.x(), 3.0f, kTol);
  EXPECT_NEAR(c.y(), 4.0f, kTol);
}

TEST(FrameTransforms, WorldLocalRoundTrip)
{
  float yaw = 0.8f;
  auto local = world_to_local(5.0f, 6.0f, yaw);
  auto world = local_to_world(local.x(), local.y(), yaw);
  EXPECT_NEAR(world.x(), 5.0f, kTol);
  EXPECT_NEAR(world.y(), 6.0f, kTol);
}

TEST(FrameTransforms, LocalToWorldIsInverse)
{
  float yaw = -0.5f;
  auto a = world_to_local(2.0f, 3.0f, yaw);
  auto b = local_to_world(a.x(), a.y(), yaw);
  EXPECT_NEAR(b.x(), 2.0f, kTol);
  EXPECT_NEAR(b.y(), 3.0f, kTol);
}

TEST(FrameTransforms, ZoneOriginToWorld)
{
  // With zero heading and zero start_yaw, origin should be unchanged
  auto origin = zone_origin_to_world(10.0f, 20.0f, 0.0f, 0.0f);
  EXPECT_NEAR(origin.x(), 10.0f, kTol);
  EXPECT_NEAR(origin.y(), 20.0f, kTol);
}

TEST(FrameTransforms, ZoneOriginWithHeading)
{
  // 90 degree heading should swap x/y
  auto origin = zone_origin_to_world(10.0f, 0.0f, kPi / 2.0f, 0.0f);
  EXPECT_NEAR(origin.x(), 0.0f, kTol);
  EXPECT_NEAR(origin.y(), 10.0f, kTol);
}
