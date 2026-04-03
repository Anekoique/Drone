#include "drone/mission/mission_types.hpp"

#include <gtest/gtest.h>

#include <cmath>

using drone::mission::MissionConfig;

TEST(MissionConfig, LoadFromYaml)
{
  // Use the project's actual config files
  auto cfg = MissionConfig::load("mission.yaml", "airdrop.yaml", "landing.yaml");

  // Mission fields
  EXPECT_FLOAT_EQ(cfg.heading_compass_deg, 349.0f);
  EXPECT_NEAR(cfg.heading_compass_rad, 349.0f * static_cast<float>(M_PI) / 180.0f, 1e-4f);

  // Shot zone
  EXPECT_FLOAT_EQ(cfg.shot_zone.dx, 0.0f);
  EXPECT_FLOAT_EQ(cfg.shot_zone.dy, 30.0f);
  EXPECT_FLOAT_EQ(cfg.shot_zone.altitude, 4.5f);
  EXPECT_FLOAT_EQ(cfg.shot_zone.altitude_surround, 3.0f);
  EXPECT_FLOAT_EQ(cfg.shot_zone.altitude_low, 1.8f);

  // Recon zone
  EXPECT_NEAR(cfg.recon_zone.dy, 54.7f, 0.1f);
  EXPECT_FLOAT_EQ(cfg.recon_zone.altitude, 3.5f);

  // Servo
  EXPECT_FLOAT_EQ(cfg.servo_open_pwm, 1980.0f);
  EXPECT_FLOAT_EQ(cfg.servo_close_pwm, 1200.0f);
  EXPECT_TRUE(cfg.prefer_large_target);

  // Airdrop fields
  EXPECT_NEAR(cfg.airdrop.pid.p, 0.190f, 0.001f);
  EXPECT_FLOAT_EQ(cfg.airdrop.radius, 0.08f);
  EXPECT_NEAR(cfg.airdrop.limits.speed_max_xy, 0.7f, 0.01f);

  // Landing fields
  EXPECT_NEAR(cfg.landing.pid.p, 0.4f, 0.01f);
  EXPECT_FLOAT_EQ(cfg.landing.scout_halt, 2.5f);
  EXPECT_FLOAT_EQ(cfg.landing.tar_z, 0.2f);

  // Waypoints should be populated
  EXPECT_EQ(cfg.shot_zone.waypoints.size(), 12u);
  EXPECT_EQ(cfg.recon_zone.waypoints.size(), 5u);
}

TEST(MissionConfig, HeadingRealDefaultsToCompass)
{
  // mission.yaml does not have headingangle_real, so it should default to compass
  auto cfg = MissionConfig::load("mission.yaml", "airdrop.yaml", "landing.yaml");
  EXPECT_NEAR(cfg.heading_real_rad, cfg.heading_compass_rad, 1e-4f);
}
