#pragma once

#include "drone/perception/detection_types.hpp"

#include <vector>

namespace drone
{

/// Find cluster centers from target samples using k-means (k=3).
/// The competition field has exactly 3 containers — k is fixed by design.
/// Preserves diameter for shot-order ranking (sorted largest first).
/// May return fewer than 3 clusters if samples are insufficient.
std::vector<Target> find_cluster_centers(const std::vector<TargetSample> & samples);

}  // namespace drone
