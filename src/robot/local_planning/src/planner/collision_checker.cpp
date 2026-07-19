#include "planning/planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kInfDistanceSqCells = 1.0e20;

int gridIndex(int row, int col, int width)
{
  return row * width + col;
}

bool pointToGridCell(const Point & p, const OccupancyGrid & grid, int & row, int & col)
{
  col = static_cast<int>(std::floor((p.x - grid.origin.x) / grid.resolution));
  row = static_cast<int>(std::floor((p.y - grid.origin.y) / grid.resolution));
  return col >= 0 && col < grid.width && row >= 0 && row < grid.height;
}

// https://hellorob.org/files/lectures/fast_euclidean_dt.pdf
void distanceTransform1d(
  const std::vector<double> & source_distance_sq,
  std::vector<double> & transformed_distance_sq)
{
  const int sample_count = static_cast<int>(source_distance_sq.size());
  transformed_distance_sq.assign(source_distance_sq.size(), kInfDistanceSqCells);

  std::vector<int> envelope_sources(static_cast<size_t>(sample_count), 0);
  std::vector<double> envelope_start_positions(static_cast<size_t>(sample_count) + 1, 0.0);
  int envelope_back = -1;

  for (int source_index = 0; source_index < sample_count; ++source_index) {
    const bool has_source =
      source_distance_sq[static_cast<size_t>(source_index)] < kInfDistanceSqCells;

    if (has_source) {
      double new_start_position = -std::numeric_limits<double>::infinity();
      bool source_fits_envelope = false;
      while (envelope_back >= 0 && !source_fits_envelope) {
        const int previous_source = envelope_sources[static_cast<size_t>(envelope_back)];
        const double source_sq =
          static_cast<double>(source_index) * static_cast<double>(source_index);
        const double previous_source_sq =
          static_cast<double>(previous_source) * static_cast<double>(previous_source);
        new_start_position =
          ((source_distance_sq[static_cast<size_t>(source_index)] + source_sq) -
          (source_distance_sq[static_cast<size_t>(previous_source)] + previous_source_sq)) /
          (2.0 * static_cast<double>(source_index - previous_source));

        source_fits_envelope =
          new_start_position > envelope_start_positions[static_cast<size_t>(envelope_back)];
        if (!source_fits_envelope) {
          --envelope_back;
        }
      }

      ++envelope_back;
      envelope_sources[static_cast<size_t>(envelope_back)] = source_index;
      envelope_start_positions[static_cast<size_t>(envelope_back)] = new_start_position;
      envelope_start_positions[static_cast<size_t>(envelope_back + 1)] =
        std::numeric_limits<double>::infinity();
    }
  }

  if (envelope_back < 0) {
    return;
  }

  int envelope_index = 0;
  for (int query_index = 0; query_index < sample_count; ++query_index) {
    while (envelope_start_positions[static_cast<size_t>(envelope_index + 1)] <
      static_cast<double>(query_index))
    {
      ++envelope_index;
    }
    const int source_index = envelope_sources[static_cast<size_t>(envelope_index)];
    const int dx = query_index - source_index;
    const double dx_sq = static_cast<double>(dx) * static_cast<double>(dx);
    transformed_distance_sq[static_cast<size_t>(query_index)] =
      dx_sq + source_distance_sq[static_cast<size_t>(source_index)];
  }
}

} // namespace

CollisionChecker::CollisionChecker(const LocalFrenetPlannerConfig & config)
: config_(config)
{
}

void CollisionChecker::buildClearanceCache(OccupancyGrid & grid) const
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= kEpsilon) {
    grid.obstacle_distance_m.clear();
    grid.has_clearance_cache = false;
    return;
  }

  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  if (grid.data.size() < cell_count) {
    grid.obstacle_distance_m.clear();
    grid.has_clearance_cache = false;
    return;
  }

  std::vector<double> row_distance_sq(cell_count, kInfDistanceSqCells);
  std::vector<double> distance_sq(cell_count, kInfDistanceSqCells);
  std::vector<double> f(static_cast<size_t>(std::max(grid.width, grid.height)));
  std::vector<double> d;

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const int index = gridIndex(row, col, grid.width);
      f[static_cast<size_t>(col)] =
        grid.data[static_cast<size_t>(index)] >= config_.occupied_threshold ?
        0.0 : kInfDistanceSqCells;
    }

    f.resize(static_cast<size_t>(grid.width));
    distanceTransform1d(f, d);
    for (int col = 0; col < grid.width; ++col) {
      row_distance_sq[static_cast<size_t>(gridIndex(row, col, grid.width))] =
        d[static_cast<size_t>(col)];
    }
    f.resize(static_cast<size_t>(std::max(grid.width, grid.height)));
  }

  f.resize(static_cast<size_t>(grid.height));
  for (int col = 0; col < grid.width; ++col) {
    for (int row = 0; row < grid.height; ++row) {
      f[static_cast<size_t>(row)] =
        row_distance_sq[static_cast<size_t>(gridIndex(row, col, grid.width))];
    }

    distanceTransform1d(f, d);
    for (int row = 0; row < grid.height; ++row) {
      distance_sq[static_cast<size_t>(gridIndex(row, col, grid.width))] =
        d[static_cast<size_t>(row)];
    }
  }

  grid.obstacle_distance_m.resize(cell_count);
  for (size_t i = 0; i < cell_count; ++i) {
    if (distance_sq[i] >= kInfDistanceSqCells) {
      grid.obstacle_distance_m[i] = std::numeric_limits<float>::infinity();
    } else {
      grid.obstacle_distance_m[i] =
        static_cast<float>(std::sqrt(distance_sq[i]) * grid.resolution);
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

  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  if (grid.has_clearance_cache && grid.obstacle_distance_m.size() >= cell_count) {
    const double cell_half_diagonal = 0.5 * std::sqrt(2.0) * grid.resolution;
    bool has_soft_inflation = false;
    for (const Point & center : circle_centers) {
      int center_row = 0;
      int center_col = 0;
      if (!pointToGridCell(center, grid, center_row, center_col)) {
        return CollisionStatus::FREE;
      }

      const size_t center_index = static_cast<size_t>(
        gridIndex(center_row, center_col, grid.width));
      const double clearance_m =
        static_cast<double>(grid.obstacle_distance_m[center_index]) -
        cell_half_diagonal - collision_radius_m;
      if (clearance_m <= 0.0) {
        return CollisionStatus::COLLISION;
      }

      if (clearance_m <= soft_inflation_distance_m) {
        has_soft_inflation = true;
      }
    }

    return has_soft_inflation ? CollisionStatus::SOFT_INFLATION : CollisionStatus::FREE;
  }

  // Check hard collision first (smaller bounding box so an immediate exit)
  for (const Point & center : circle_centers) {
    int center_row = 0;
    int center_col = 0;
    if (!pointToGridCell(center, grid, center_row, center_col)) {
      return CollisionStatus::FREE;
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

  // If no hard collision was found, scan the outer bounding box for soft inflation
  for (const Point & center : circle_centers) {
    int center_row = 0;
    int center_col = 0;
    if (!pointToGridCell(center, grid, center_row, center_col)) {
      return CollisionStatus::FREE;
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

CollisionCheckResult CollisionChecker::collisionCheck(
  const Point & p,
  double heading,
  const OccupancyGrid & grid) const
{
  const CollisionStatus status = collisionStatus(p, heading, grid);
  if (status == CollisionStatus::OUT_OF_GRID ||
    status == CollisionStatus::GEOMETRY_CONSTRAINT)
  {
    return {status, -std::numeric_limits<double>::infinity()};
  }

  const double collision_radius_m = std::max(0.0, config_.collision_circle_radius_m);
  const Point circle_centers[] = {
    p,
    {
      p.x + config_.front_collision_circle_offset_m * std::cos(heading),
      p.y + config_.front_collision_circle_offset_m * std::sin(heading),
      p.velocity
    }
  };
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);

  if (grid.has_clearance_cache && grid.obstacle_distance_m.size() >= cell_count) {
    const double cell_half_diagonal = 0.5 * std::sqrt(2.0) * grid.resolution;
    for (const Point & center : circle_centers) {
      int row = 0;
      int col = 0;
      if (!pointToGridCell(center, grid, row, col)) {
        continue;
      }
      const double clearance_m =
        static_cast<double>(grid.obstacle_distance_m[static_cast<size_t>(
        gridIndex(row, col, grid.width))]) - cell_half_diagonal - collision_radius_m;
      minimum_clearance_m = std::min(minimum_clearance_m, clearance_m);
    }
  } else {
    // The planner node normally supplies the distance cache. This exact fallback
    // keeps direct users correct without adding another approximate soft cost.
    for (const Point & center : circle_centers) {
      for (int row = 0; row < grid.height; ++row) {
        for (int col = 0; col < grid.width; ++col) {
          const size_t index = static_cast<size_t>(gridIndex(row, col, grid.width));
          if (index >= grid.data.size() ||
            grid.data[index] < config_.occupied_threshold)
          {
            continue;
          }
          const double cell_x =
            grid.origin.x + (static_cast<double>(col) + 0.5) * grid.resolution;
          const double cell_y =
            grid.origin.y + (static_cast<double>(row) + 0.5) * grid.resolution;
          minimum_clearance_m = std::min(
            minimum_clearance_m,
            std::hypot(cell_x - center.x, cell_y - center.y) - collision_radius_m);
        }
      }
    }
  }

  return {status, minimum_clearance_m};
}

} // namespace local_planning
