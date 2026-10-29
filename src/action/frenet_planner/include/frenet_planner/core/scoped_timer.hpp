#ifndef FRENET_PLANNER_CORE_SCOPED_TIMER_HPP
#define FRENET_PLANNER_CORE_SCOPED_TIMER_HPP

#include <chrono>

namespace frenet_planner
{

// Adds elapsed milliseconds to sink on destruction. Repeated or nested scopes
// accumulate, so extra timers can be added later without a start/stop sandwich.
struct ScopedTimer
{
  double & sink;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  ScopedTimer(double & sink) : sink(sink) {}
  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer & operator=(const ScopedTimer &) = delete;

  ~ScopedTimer()
  {
    sink += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_CORE_SCOPED_TIMER_HPP
