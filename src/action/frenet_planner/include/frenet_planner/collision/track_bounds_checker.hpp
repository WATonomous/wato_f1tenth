#ifndef FRENET_PLANNER_COLLISION_TRACK_BOUNDS_CHECKER_HPP
#define FRENET_PLANNER_COLLISION_TRACK_BOUNDS_CHECKER_HPP

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/reference/raceline_reference.hpp"

#include <vector>

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
  TrackBoundsChecker(
    const RacelineReference & reference,
    VehicleGeometry vehicle_geometry);

  TrackBoundsCheckResult check(
    const std::vector<CurveSample> & path,
    const OccupancyGrid & grid) const;

private:
  const RacelineReference & reference_;
  VehicleGeometry vehicle_geometry_;
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_COLLISION_TRACK_BOUNDS_CHECKER_HPP
