#ifndef LOCAL_PLANNING_COLLISION_TRACK_BOUNDS_CHECKER_HPP
#define LOCAL_PLANNING_COLLISION_TRACK_BOUNDS_CHECKER_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <vector>

namespace local_planning
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

}  // namespace local_planning

#endif  // LOCAL_PLANNING_COLLISION_TRACK_BOUNDS_CHECKER_HPP
