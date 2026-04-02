#pragma once

#include <chrono>

namespace drone
{

class Timer
{
public:
  Timer() : start_(clock::now()) {}

  void reset() { start_ = clock::now(); }

  double elapsed() const
  {
    if (start_ == clock::time_point{}) return 0.0;
    return seconds(clock::now() - start_).count();
  }

  void set_timepoint() { timepoint_ = clock::now(); }

  double timepoint_elapsed() const
  {
    if (timepoint_ == clock::time_point{}) return 0.0;
    return seconds(clock::now() - timepoint_).count();
  }

  void invalidate() { start_ = clock::time_point{}; }

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
