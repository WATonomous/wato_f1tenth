#include "planning/planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kFrontCollisionCircleOffsetM = 0.26;

} // namespace

CollisionChecker::CollisionChecker(const LocalFrenetPlannerConfig & config)
: config_(config)
{
}

CollisionStatus CollisionChecker::collisionStatus(
  const Point & p,
  double heading,
  const OccupancyGrid & grid) const
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= kEpsilon) {
    return CollisionStatus::OUT_OF_GRID;
  }

  const int inflation_cells =
    std::max(
    0,
    static_cast<int>(std::ceil(config_.obstacle_inflation_distance_m / grid.resolution)));
  const double inflation_sq = config_.obstacle_inflation_distance_m *
    config_.obstacle_inflation_distance_m;
  const Point circle_centers[] = {
    p,
    {
      p.x + kFrontCollisionCircleOffsetM * std::cos(heading),
      p.y + kFrontCollisionCircleOffsetM * std::sin(heading),
      p.velocity
    }
  };

  for (const Point & center : circle_centers) {
    const int center_col = static_cast<int>((center.x - grid.origin.x) / grid.resolution);
    const int center_row = static_cast<int>((center.y - grid.origin.y) / grid.resolution);
    if (center_col < 0 || center_col >= grid.width || center_row < 0 || center_row >= grid.height) {
      return CollisionStatus::OUT_OF_GRID;
    }

    for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
      for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
        const int row = center_row + dr;
        const int col = center_col + dc;
        if (row < 0 || row >= grid.height || col < 0 || col >= grid.width) {
          continue;
        }

        const double cell_x = grid.origin.x + (static_cast<double>(col) + 0.5) * grid.resolution;
        const double cell_y = grid.origin.y + (static_cast<double>(row) + 0.5) * grid.resolution;
        const double dx = cell_x - center.x;
        const double dy = cell_y - center.y;
        if (dx * dx + dy * dy > inflation_sq) {
          continue;
        }
        if (grid.data[static_cast<size_t>(row * grid.width + col)] >= config_.occupied_threshold) {
          return CollisionStatus::COLLISION;
        }
      }
    }
  }

  return CollisionStatus::FREE;
}

} // namespace local_planning
