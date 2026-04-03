// Copyright (c) 2024-2026 HDU-DXY-Team
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <chrono>

namespace drone
{

/// Monotonic timer for measuring elapsed time and marking timepoints.
class Timer
{
public:
  Timer() : start_(clock::now()) {}

  /// Reset the start time to now.
  void reset() { start_ = clock::now(); }

  /// Seconds elapsed since construction or last reset. Returns 0 if invalidated.
  double elapsed() const
  {
    if (start_ == clock::time_point{}) return 0.0;
    return seconds(clock::now() - start_).count();
  }

  /// Record a secondary timepoint (independent of start).
  void set_timepoint() { timepoint_ = clock::now(); }

  /// Seconds elapsed since last set_timepoint(). Returns 0 if never set.
  double timepoint_elapsed() const
  {
    if (timepoint_ == clock::time_point{}) return 0.0;
    return seconds(clock::now() - timepoint_).count();
  }

  /// Mark timer as invalid; elapsed() will return 0.
  void invalidate() { start_ = clock::time_point{}; }

  /// Set start time so that elapsed() returns approximately seconds_ago.
  /// @param seconds_ago Desired elapsed time in seconds.
  void set_elapsed(double seconds_ago)
  {
    start_ = clock::now() - std::chrono::duration_cast<clock::duration>(
                              std::chrono::duration<double>(seconds_ago));
  }

private:
  using clock = std::chrono::steady_clock;
  using seconds = std::chrono::duration<double>;

  clock::time_point start_;
  clock::time_point timepoint_{};
};

}  // namespace drone
