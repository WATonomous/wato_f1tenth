
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

/*
  if we cant see a point we check the track bounds for that frenet s value
*/

#include "frenet_planner/collision/track_bounds_checker.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

#include "frenet_planner/core/geometry.hpp"

namespace frenet_planner
{
namespace
{
constexpr double kTrackBoundsToleranceM = kGridEps;

bool cellKnown(const OccupancyGrid & grid, const Point & p)
{
  const auto index = grid.cellAt(p);
  return index && grid.data[*index] >= 0;
}

bool footprintUnseen(const CurveSample & sample, const OccupancyGrid & grid, const VehicleGeometry & vehicle)
{
  const Point body(sample.x, sample.y);
  const Point front(
    sample.x + vehicle.front_circle_offset_m * std::cos(sample.heading),
    sample.y + vehicle.front_circle_offset_m * std::sin(sample.heading));
  return !cellKnown(grid, body) || !cellKnown(grid, front);
}
}  // namespace

TrackBoundsChecker::TrackBoundsChecker(const RacelineReference & reference, VehicleGeometry vehicle_geometry)
: reference_(reference)
, vehicle_geometry_(vehicle_geometry)
{}

TrackBoundsCheckResult TrackBoundsChecker::check(
  const std::vector<CurveSample> & path, const OccupancyGrid & grid) const
{
  TrackBoundsCheckResult result;
  if (!reference_.trackWidthsValid()) {
    return result;
  }

  for (const CurveSample & sample : path) {
    if (!footprintUnseen(sample, grid, vehicle_geometry_)) {
      continue;
    }

    const SustainableBounds bounds = reference_.rawBounds(sample.raceline_s);
    const double cap = sample.d >= 0.0 ? bounds.left_magnitude : bounds.right_magnitude;
    if (std::abs(sample.d) > cap + kTrackBoundsToleranceM) {
      result.ok = false;
      return result;
    }
  }
  return result;
}

}  // namespace frenet_planner
