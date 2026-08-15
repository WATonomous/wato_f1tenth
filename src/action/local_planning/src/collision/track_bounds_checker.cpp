#include "local_planning/collision/track_bounds_checker.hpp"

#include <cmath>
#include <cstddef>
#include <optional>

namespace local_planning
{
namespace
{
constexpr double kTrackBoundsToleranceM = 1e-6;

bool cellKnown(const OccupancyGrid & grid, const Point & p)
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0) {
    return false;
  }
  const int col = static_cast<int>(std::floor((p.x - grid.origin.x) / grid.resolution));
  const int row = static_cast<int>(std::floor((p.y - grid.origin.y) / grid.resolution));
  if (col < 0 || col >= grid.width || row < 0 || row >= grid.height) {
    return false;
  }
  const std::size_t index =
    static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
    static_cast<std::size_t>(col);
  if (index >= grid.data.size()) {
    return false;
  }
  return grid.data[index] >= 0;
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

  std::optional<double> previous_s;
  for (const CurveSample & sample : path) {
    if (!footprintUnseen(sample, grid, vehicle_geometry_)) {
      previous_s = sample.raceline_s;
      continue;
    }

    const Point p(sample.x, sample.y);
    bool converged = false;
    double refined_s = sample.raceline_s;
    double d = reference_.lateralOffsetAt(p, sample.raceline_s, &converged, &refined_s);
    ++result.station_hint_samples;
    if (!converged && previous_s) {
      d = reference_.lateralOffsetAt(p, *previous_s, &converged, &refined_s);
      ++result.station_hint_samples;
    }
    if (!converged) {
      ++result.station_hint_fallbacks;
      const Projection projection = reference_.project(p, sample.raceline_s);
      d = projection.d;
      refined_s = projection.s;
    }
    previous_s = refined_s;

    const SustainableBounds bounds = reference_.rawBounds(refined_s);
    const double cap = d >= 0.0 ? bounds.left_magnitude : bounds.right_magnitude;
    if (std::abs(d) > cap + kTrackBoundsToleranceM) {
      result.ok = false;
      return result;
    }
  }
  return result;
}

}  // namespace local_planning
