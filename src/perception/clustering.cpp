#include "drone/perception/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace drone
{

namespace
{

struct NormalizedSample
{
  double x = 0, y = 0, d = 0;
  size_t original_index = 0;
};

double compute_mean(const std::vector<double> & data)
{
  if (data.empty()) return 0.0;
  return std::accumulate(data.begin(), data.end(), 0.0) / static_cast<double>(data.size());
}

double compute_std_dev(const std::vector<double> & data, double mean)
{
  if (data.empty()) return 0.0;
  double sum_sq = 0.0;
  for (double v : data) sum_sq += (v - mean) * (v - mean);
  return std::sqrt(sum_sq / static_cast<double>(data.size()));
}

std::vector<NormalizedSample> normalize(const std::vector<TargetSample> & samples)
{
  std::vector<double> xs, ys, ds;
  for (const auto & s : samples) {
    xs.push_back(s.position.x());
    ys.push_back(s.position.y());
    ds.push_back(s.diameter);
  }

  double mx = compute_mean(xs), my = compute_mean(ys), md = compute_mean(ds);
  double sx = compute_std_dev(xs, mx), sy = compute_std_dev(ys, my), sd = compute_std_dev(ds, md);

  std::vector<NormalizedSample> result;
  for (size_t i = 0; i < samples.size(); ++i) {
    NormalizedSample ns;
    ns.x = (sx > 0) ? (xs[i] - mx) / sx : 0;
    ns.y = (sy > 0) ? (ys[i] - my) / sy : 0;
    ns.d = (sd > 0) ? (ds[i] - md) / sd : 0;
    ns.original_index = i;
    result.push_back(ns);
  }
  return result;
}

double distance_3d(const NormalizedSample & a, const NormalizedSample & b)
{
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.d - b.d, 2));
}

}  // namespace

std::vector<Target> find_cluster_centers(const std::vector<TargetSample> & samples)
{
  if (samples.size() < 3) {
    // Not enough samples — return each as its own cluster
    std::vector<Target> result;
    for (size_t i = 0; i < samples.size(); ++i) {
      Target t;
      t.position = samples[i].position;
      t.diameter = samples[i].diameter;
      t.cluster_id = static_cast<int>(i);
      result.push_back(t);
    }
    return result;
  }

  auto normalized = normalize(samples);

  // Initialize 3 cluster centers (first, middle, last)
  constexpr int k = 3;
  NormalizedSample centers[k];
  centers[0] = normalized[0];
  centers[1] = normalized[normalized.size() / 2];
  centers[2] = normalized[normalized.size() - 1];

  std::vector<int> assignments(normalized.size(), 0);

  // K-means iterations
  for (int iter = 0; iter < 50; ++iter) {
    bool changed = false;

    // Assign each point to nearest center
    for (size_t i = 0; i < normalized.size(); ++i) {
      int best = 0;
      double best_dist = distance_3d(normalized[i], centers[0]);
      for (int c = 1; c < k; ++c) {
        double d = distance_3d(normalized[i], centers[c]);
        if (d < best_dist) {
          best_dist = d;
          best = c;
        }
      }
      if (assignments[i] != best) {
        assignments[i] = best;
        changed = true;
      }
    }

    if (!changed) break;

    // Recompute centers
    for (int c = 0; c < k; ++c) {
      double sx = 0, sy = 0, sd = 0;
      int count = 0;
      for (size_t i = 0; i < normalized.size(); ++i) {
        if (assignments[i] == c) {
          sx += normalized[i].x;
          sy += normalized[i].y;
          sd += normalized[i].d;
          count++;
        }
      }
      if (count > 0) {
        centers[c].x = sx / count;
        centers[c].y = sy / count;
        centers[c].d = sd / count;
      }
    }
  }

  // Compute cluster centers in original space
  std::vector<Target> result;
  for (int c = 0; c < k; ++c) {
    Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
    double diam_sum = 0;
    int count = 0;
    for (size_t i = 0; i < normalized.size(); ++i) {
      if (assignments[i] == c) {
        pos_sum += samples[normalized[i].original_index].position;
        diam_sum += samples[normalized[i].original_index].diameter;
        count++;
      }
    }
    if (count > 0) {
      Target t;
      t.position = pos_sum / count;
      t.diameter = diam_sum / count;
      t.cluster_id = c;
      result.push_back(t);
    }
  }

  // Sort by diameter (largest first) for shot-order ranking
  std::sort(result.begin(), result.end(), [](const Target & a, const Target & b) {
    return a.diameter > b.diameter;
  });

  return result;
}

}  // namespace drone
