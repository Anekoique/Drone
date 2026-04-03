// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0
/// @file test_clustering.cpp
/// @brief Unit tests for k-means target clustering.

#include "drone/perception/clustering.hpp"

#include <gtest/gtest.h>

using drone::Target;
using drone::TargetSample;

TEST(ClusteringTest, EmptyInput)
{
  auto result = drone::find_cluster_centers({});
  EXPECT_TRUE(result.empty());
}

TEST(ClusteringTest, SingleSample)
{
  TargetSample s;
  s.position = Eigen::Vector3d(1, 2, 0);
  s.diameter = 0.15;
  auto result = drone::find_cluster_centers({s});
  ASSERT_EQ(result.size(), 1);
  EXPECT_NEAR(result[0].position.x(), 1.0, 0.01);
  EXPECT_NEAR(result[0].diameter, 0.15, 0.01);
}

TEST(ClusteringTest, ThreeDistinctClusters)
{
  std::vector<TargetSample> samples;

  // Cluster around (0, 0), diameter 0.15 (small)
  for (int i = 0; i < 15; i++) {
    TargetSample s;
    s.position = Eigen::Vector3d(0.02 * i, 0.02 * i, 0);
    s.diameter = 0.14 + 0.01 * (i % 3);
    samples.push_back(s);
  }

  // Cluster around (10, 0), diameter 0.20 (medium)
  for (int i = 0; i < 15; i++) {
    TargetSample s;
    s.position = Eigen::Vector3d(10.0 + 0.02 * i, 0.02 * i, 0);
    s.diameter = 0.19 + 0.01 * (i % 3);
    samples.push_back(s);
  }

  // Cluster around (20, 0), diameter 0.25 (large)
  for (int i = 0; i < 15; i++) {
    TargetSample s;
    s.position = Eigen::Vector3d(20.0 + 0.02 * i, 0.02 * i, 0);
    s.diameter = 0.24 + 0.01 * (i % 3);
    samples.push_back(s);
  }

  auto result = drone::find_cluster_centers(samples);
  ASSERT_EQ(result.size(), 3);

  // Should be sorted by diameter (largest first)
  EXPECT_GT(result[0].diameter, result[1].diameter);
  EXPECT_GT(result[1].diameter, result[2].diameter);

  // Check approximate center positions (wider tolerance for k-means)
  EXPECT_NEAR(result[0].position.x(), 20.0, 2.0);  // largest
  EXPECT_NEAR(result[1].position.x(), 10.0, 2.0);  // medium
  EXPECT_NEAR(result[2].position.x(), 0.0, 2.0);   // smallest
}

TEST(ClusteringTest, DiameterPreserved)
{
  std::vector<TargetSample> samples;
  for (int i = 0; i < 5; i++) {
    TargetSample s;
    s.position = Eigen::Vector3d(0, 0, 0);
    s.diameter = 0.20;
    samples.push_back(s);
  }
  for (int i = 0; i < 5; i++) {
    TargetSample s;
    s.position = Eigen::Vector3d(5, 0, 0);
    s.diameter = 0.15;
    samples.push_back(s);
  }
  for (int i = 0; i < 5; i++) {
    TargetSample s;
    s.position = Eigen::Vector3d(10, 0, 0);
    s.diameter = 0.25;
    samples.push_back(s);
  }

  auto result = drone::find_cluster_centers(samples);
  // Diameters should be approximately preserved
  for (const auto & t : result) {
    EXPECT_GT(t.diameter, 0.1);
    EXPECT_LT(t.diameter, 0.3);
  }
}
