// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

  explicit ScopedTimer(double & sink)
  : sink(sink)
  {}

  ScopedTimer(const ScopedTimer &) = delete;
  ScopedTimer & operator=(const ScopedTimer &) = delete;

  ~ScopedTimer()
  {
    sink += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  }
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_CORE_SCOPED_TIMER_HPP
