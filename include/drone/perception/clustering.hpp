// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "drone/perception/detection_types.hpp"

#include <vector>

namespace drone
{

/// Find cluster centers from target samples using k-means (k=3).
/// The competition field has exactly 3 containers -- k is fixed by design.
/// Preserves diameter for shot-order ranking (sorted largest first).
/// @param samples World-frame target samples to cluster.
/// @return Up to 3 cluster centers, sorted by diameter (largest first).
std::vector<Target> find_cluster_centers(const std::vector<TargetSample> & samples);

}  // namespace drone
