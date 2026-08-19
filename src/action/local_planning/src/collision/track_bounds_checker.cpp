
/*
  if we cant see a point we check the track bounds for that frenet s value
*/




#include "local_planning/collision/track_bounds_checker.hpp"

#include "local_planning/core/geometry.hpp"

#include <cmath>
#include <cstddef>



namespace local_planning
{
namespace
{
constexpr double kTrackBoundsToleranceM = kGridEps;

bool cellKnown(const OccupancyGrid & grid, const Point & p)
{
  const auto index = grid.cellAt(p);
  return index && grid.data[*index] >= 0;
}

bool footprintUnseen(
  const CurveSample & sample,
  const OccupancyGrid & grid,
  const VehicleGeometry & vehicle)
{
  const Point body(sample.x, sample.y);
  const Point front(
    sample.x + vehicle.front_circle_offset_m * std::cos(sample.heading),
    sample.y + vehicle.front_circle_offset_m * std::sin(sample.heading));
  return !cellKnown(grid, body) || !cellKnown(grid, front);
}
}  // namespace

TrackBoundsChecker::TrackBoundsChecker(
  const RacelineReference & reference,
  VehicleGeometry vehicle_geometry)
: reference_(reference), vehicle_geometry_(vehicle_geometry)
{
}

TrackBoundsCheckResult TrackBoundsChecker::check(
  const std::vector<CurveSample> & path,
  const OccupancyGrid & grid) const
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

}  // namespace local_planning
