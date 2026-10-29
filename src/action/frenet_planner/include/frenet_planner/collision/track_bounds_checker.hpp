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

#ifndef FRENET_PLANNER_COLLISION_TRACK_BOUNDS_CHECKER_HPP
#define FRENET_PLANNER_COLLISION_TRACK_BOUNDS_CHECKER_HPP

#include <vector>

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/reference/raceline_reference.hpp"

namespace frenet_planner
{

struct TrackBoundsCheckResult
{
  bool ok = true;
};

// Width-table prior for unknown/out-of-grid samples; known cells skip this check.
class TrackBoundsChecker
{
public:
  TrackBoundsChecker(const RacelineReference & reference, VehicleGeometry vehicle_geometry);

  TrackBoundsCheckResult check(const std::vector<CurveSample> & path, const OccupancyGrid & grid) const;

private:
  const RacelineReference & reference_;
  VehicleGeometry vehicle_geometry_;
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_COLLISION_TRACK_BOUNDS_CHECKER_HPP
