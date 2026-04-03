#include "drone/control/limits.hpp"

#include <gtest/gtest.h>

using drone::control::Limits;
using drone::control::load_limits;

TEST(Limits, DefaultValues)
{
  Limits lim;
  EXPECT_FLOAT_EQ(lim.speed_max_xy, 2.0f);
  EXPECT_FLOAT_EQ(lim.speed_max_z, 1.0f);
  EXPECT_FLOAT_EQ(lim.speed_max_yaw, 0.3f);
  EXPECT_FLOAT_EQ(lim.accel_max_xy, 1.2f);
  EXPECT_FLOAT_EQ(lim.accel_max_z, 1.2f);
  EXPECT_FLOAT_EQ(lim.accel_max_yaw, 0.0f);
}

TEST(Limits, LoadFromYaml)
{
  YAML::Node yaml;
  yaml["speed_max_xy"] = 4.0f;
  yaml["speed_max_z"] = 0.8f;
  yaml["speed_max_yaw"] = 0.2f;
  yaml["accel_max_xy"] = 1.5f;
  yaml["accel_max_z"] = 0.6f;
  yaml["accel_max_yaw"] = 0.1f;

  auto lim = load_limits(yaml);
  EXPECT_FLOAT_EQ(lim.speed_max_xy, 4.0f);
  EXPECT_FLOAT_EQ(lim.speed_max_z, 0.8f);
  EXPECT_FLOAT_EQ(lim.speed_max_yaw, 0.2f);
  EXPECT_FLOAT_EQ(lim.accel_max_xy, 1.5f);
  EXPECT_FLOAT_EQ(lim.accel_max_z, 0.6f);
  EXPECT_FLOAT_EQ(lim.accel_max_yaw, 0.1f);
}

TEST(Limits, LoadPartialYaml)
{
  YAML::Node yaml;
  yaml["speed_max_xy"] = 3.0f;
  // Other fields should keep defaults

  auto lim = load_limits(yaml);
  EXPECT_FLOAT_EQ(lim.speed_max_xy, 3.0f);
  EXPECT_FLOAT_EQ(lim.speed_max_z, 1.0f);  // default
}

TEST(Limits, LoadEmptyYaml)
{
  YAML::Node yaml;
  auto lim = load_limits(yaml);
  // All defaults
  EXPECT_FLOAT_EQ(lim.speed_max_xy, 2.0f);
  EXPECT_FLOAT_EQ(lim.speed_max_z, 1.0f);
}

TEST(Limits, LoadNullYaml)
{
  YAML::Node yaml;
  yaml.reset();  // null node
  auto lim = load_limits(yaml);
  EXPECT_FLOAT_EQ(lim.speed_max_xy, 2.0f);
}
