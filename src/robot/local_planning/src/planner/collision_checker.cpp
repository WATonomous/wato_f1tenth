#include "planning/planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;

} // namespace

CollisionChecker::CollisionChecker(const LocalFrenetPlannerConfig & config)
: config_(config)
{
}

void CollisionChecker::buildClearanceCache(OccupancyGrid & grid) const
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= kEpsilon) {
    grid.definitely_blocked_mask.clear();
    grid.needs_exact_check_mask.clear();
    grid.has_clearance_cache = false;
    return;
  }

  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  grid.definitely_blocked_mask.assign(cell_count, 0);
  grid.needs_exact_check_mask.assign(cell_count, 0);

  const double cell_half_diagonal = 0.5 * std::sqrt(2.0) * grid.resolution;
  const double blocked_radius = std::max(
    0.0, config_.collision_circle_radius_m - cell_half_diagonal);
  const double exact_check_radius = std::max(
    blocked_radius,
    config_.collision_circle_radius_m +
    config_.soft_inflation_distance_m + cell_half_diagonal);

  auto makeDiskOffsets = [&](double radius_m) {
      std::vector<std::pair<int, int>> offsets;
      const int max_cells = std::max(
        0, static_cast<int>(std::ceil(radius_m / grid.resolution)));
      const double radius_sq = radius_m * radius_m;
      for (int dr = -max_cells; dr <= max_cells; ++dr) {
        for (int dc = -max_cells; dc <= max_cells; ++dc) {
          const double dx = static_cast<double>(dc) * grid.resolution;
          const double dy = static_cast<double>(dr) * grid.resolution;
          if (dx * dx + dy * dy <= radius_sq) {
            offsets.emplace_back(dr, dc);
          }
        }
      }
      return offsets;
    };

  const std::vector<std::pair<int, int>> blocked_offsets = makeDiskOffsets(blocked_radius);
  const std::vector<std::pair<int, int>> exact_offsets = makeDiskOffsets(exact_check_radius);

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const size_t source_index = static_cast<size_t>(row * grid.width + col);
      if (grid.data[source_index] < config_.occupied_threshold) {
        continue;
      }

      for (const auto & offset : blocked_offsets) {
        const int masked_row = row + offset.first;
        const int masked_col = col + offset.second;
        if (masked_row < 0 || masked_row >= grid.height || masked_col < 0 || masked_col >= grid.width) {
          continue;
        }

        grid.definitely_blocked_mask[
          static_cast<size_t>(masked_row * grid.width + masked_col)] = 1;
      }

      for (const auto & offset : exact_offsets) {
        const int masked_row = row + offset.first;
        const int masked_col = col + offset.second;
        if (masked_row < 0 || masked_row >= grid.height || masked_col < 0 || masked_col >= grid.width) {
          continue;
        }

        grid.needs_exact_check_mask[
          static_cast<size_t>(masked_row * grid.width + masked_col)] = 1;
      }
    }
  }

  grid.has_clearance_cache = true;
}

CollisionStatus CollisionChecker::collisionStatus(
  const Point & p,
  double heading,
  const OccupancyGrid & grid) const
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= kEpsilon) {
    return CollisionStatus::OUT_OF_GRID;
  }

  const double collision_radius_m = std::max(0.0, config_.collision_circle_radius_m);
  const double soft_inflation_distance_m = std::max(0.0, config_.soft_inflation_distance_m);
  const double outer_radius_m = collision_radius_m + soft_inflation_distance_m;
  const int inflation_cells =
    std::max(
    0,
    static_cast<int>(std::ceil(outer_radius_m / grid.resolution)));
  const int hard_inflation_cells =
    std::max(
    0,
    static_cast<int>(std::ceil(collision_radius_m / grid.resolution)));
  const double collision_radius_sq = collision_radius_m * collision_radius_m;
  const double outer_radius_sq = outer_radius_m * outer_radius_m;
  const Point circle_centers[] = {
    p,
    {
      p.x + config_.front_collision_circle_offset_m * std::cos(heading),
      p.y + config_.front_collision_circle_offset_m * std::sin(heading),
      p.velocity
    }
  };

  if (grid.has_clearance_cache) {
    bool needs_exact_check = false;
    for (const Point & center : circle_centers) {
      const int center_col = static_cast<int>((center.x - grid.origin.x) / grid.resolution);
      const int center_row = static_cast<int>((center.y - grid.origin.y) / grid.resolution);
      if (center_col < 0 || center_col >= grid.width || center_row < 0 || center_row >= grid.height) {
        return CollisionStatus::OUT_OF_GRID;
      }

      const size_t center_index = static_cast<size_t>(center_row * grid.width + center_col);
      if (center_index >= grid.definitely_blocked_mask.size() ||
        center_index >= grid.needs_exact_check_mask.size())
      {
        needs_exact_check = true;
        continue;
      }

      if (grid.definitely_blocked_mask[center_index] != 0) {
        return CollisionStatus::COLLISION;
      }

      if (grid.needs_exact_check_mask[center_index] != 0) {
        needs_exact_check = true;
      }
    }

    if (!needs_exact_check) {
      return CollisionStatus::FREE;
    }
  }

  // Check hard collision first (smaller bounding box, immediate exit)
  for (const Point & center : circle_centers) {
    const int center_col = static_cast<int>((center.x - grid.origin.x) / grid.resolution);
    const int center_row = static_cast<int>((center.y - grid.origin.y) / grid.resolution);
    if (center_col < 0 || center_col >= grid.width || center_row < 0 || center_row >= grid.height) {
      return CollisionStatus::OUT_OF_GRID;
    }

    for (int dr = -hard_inflation_cells; dr <= hard_inflation_cells; ++dr) {
      for (int dc = -hard_inflation_cells; dc <= hard_inflation_cells; ++dc) {
        const int row = center_row + dr;
        const int col = center_col + dc;
        if (row < 0 || row >= grid.height || col < 0 || col >= grid.width) {
          continue;
        }

        if (grid.data[static_cast<size_t>(row * grid.width + col)] < config_.occupied_threshold) {
          continue;
        }

        const double cell_x = grid.origin.x + (static_cast<double>(col) + 0.5) * grid.resolution;
        const double cell_y = grid.origin.y + (static_cast<double>(row) + 0.5) * grid.resolution;
        const double distance_sq =
          (cell_x - center.x) * (cell_x - center.x) +
          (cell_y - center.y) * (cell_y - center.y);

        if (distance_sq <= collision_radius_sq) {
          return CollisionStatus::COLLISION;
        }
      }
    }
  }

  if (soft_inflation_distance_m <= 0.0) {
    return CollisionStatus::FREE;
  }

  // If no hard collision was found, quickly scan the outer bounding box for soft inflation
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

        if (grid.data[static_cast<size_t>(row * grid.width + col)] < config_.occupied_threshold) {
          continue;
        }

        const double cell_x = grid.origin.x + (static_cast<double>(col) + 0.5) * grid.resolution;
        const double cell_y = grid.origin.y + (static_cast<double>(row) + 0.5) * grid.resolution;
        const double distance_sq =
          (cell_x - center.x) * (cell_x - center.x) +
          (cell_y - center.y) * (cell_y - center.y);

        if (distance_sq <= outer_radius_sq) {
          return CollisionStatus::SOFT_INFLATION;
        }
      }
    }
  }

  return CollisionStatus::FREE;
}

} // namespace local_planning
