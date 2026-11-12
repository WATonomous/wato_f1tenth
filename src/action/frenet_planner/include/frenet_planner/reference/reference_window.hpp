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

#ifndef FRENET_PLANNER_REFERENCE_REFERENCE_WINDOW_HPP
#define FRENET_PLANNER_REFERENCE_REFERENCE_WINDOW_HPP

#include <cstddef>
#include <vector>

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/reference/raceline_reference.hpp"

namespace frenet_planner
{

// Per-cycle reference geometry on a uniform station grid; evaluated once, shared by all candidates.
class ReferenceWindow
{
public:
  // [s_start, s_start + length_m]; endpoints are exact (not snapped approximations).
  bool build(const RacelineReference & reference, double s_start, double length_m, double spacing_m);

  bool valid() const
  {
    return !samples_.empty();
  }

  std::size_t size() const
  {
    return samples_.size();
  }

  double startS() const
  {
    return s_start_;
  }

  double spacingM() const
  {
    return spacing_;
  }

  double lengthM() const
  {
    return spacing_ * static_cast<double>(samples_.size() - 1);
  }

  double sAt(std::size_t i) const
  {
    return s_start_ + spacing_ * static_cast<double>(i);
  }  // unwrapped

  std::size_t indexForS(double s) const;  // wrapping-aware nearest index, clamped

  const ReferenceGeometrySample & at(std::size_t i) const
  {
    return samples_[i];
  }

private:
  std::vector<ReferenceGeometrySample> samples_;
  const RacelineReference * reference_ = nullptr;
  double s_start_ = 0.0;
  double spacing_ = 0.0;
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_REFERENCE_REFERENCE_WINDOW_HPP
