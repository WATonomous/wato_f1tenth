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

#include "frenet_planner/reference/reference_window.hpp"

#include <algorithm>
#include <cmath>

#include "frenet_planner/core/geometry.hpp"

namespace frenet_planner
{
namespace
{
constexpr double kTolerance = kSpacingEps;
}  // namespace

/*
sample the raceline from s start to s_start + length
get position, tangent, normal, cruvature, speed along sampled points along racelien
*/
bool ReferenceWindow::build(const RacelineReference & reference, double s_start, double length_m, double spacing_m)
{
  samples_.clear();
  reference_ = nullptr;
  if (
    !reference.valid() || !std::isfinite(s_start) || !std::isfinite(length_m) || length_m <= 0.0 ||
    !std::isfinite(spacing_m) || spacing_m <= 0.0)
  {
    return false;
  }

  // Space points to the resolution of the costmap
  const auto interval_count = static_cast<std::size_t>(std::max(1.0, std::ceil(length_m / spacing_m - kTolerance)));
  s_start_ = s_start;
  spacing_ = length_m / static_cast<double>(interval_count);
  reference_ = &reference;

  samples_.resize(interval_count + 1);
  for (std::size_t i = 0; i <= interval_count; ++i) {
    samples_[i] = reference.sampleAtS(s_start_ + spacing_ * static_cast<double>(i));
  }
  return true;
}

std::size_t ReferenceWindow::indexForS(double s) const
{
  if (samples_.empty() || reference_ == nullptr || spacing_ <= 0.0) {
    return 0;
  }
  const double progress = reference_->deltaS(s_start_, s);
  const double index = std::round(progress / spacing_);
  if (!(index > 0.0)) {
    return 0;
  }
  return std::min(static_cast<std::size_t>(index), samples_.size() - 1);
}

}  // namespace frenet_planner
