#include "local_planning/collision/collision_checker.hpp"

#include "local_planning/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = kGridEps;
constexpr double kInfDistanceSqCells = 1.0e20;

int gridIndex(int row, int col, int width)
{
  return row * width + col;
}

bool gridGeometryValid(const OccupancyGrid & grid)
{
  return grid.width > 0 && grid.height > 0 && grid.resolution > kEpsilon;
}

bool euclideanTransformValid(const OccupancyGrid & grid)
{
  if (!gridGeometryValid(grid) || !grid.has_euclidean_transform) {
    return false;
  }
  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  return grid.data.size() >= cell_count && grid.obstacle_distance_m.size() >= cell_count;
}

CollisionStatus worseStatus(CollisionStatus a, CollisionStatus b)
{
  const auto rank = [](CollisionStatus status) {
      switch (status) {
        case CollisionStatus::COLLISION:
          return 3;
        case CollisionStatus::OUT_OF_GRID:
          return 2;
        case CollisionStatus::SOFT_INFLATION:
          return 1;
        case CollisionStatus::FREE:
          return 0;
      }
      return 0;
    };
  return rank(a) >= rank(b) ? a : b;
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

CollisionChecker::CollisionChecker(
  VehicleGeometry vehicle_geometry,
  GridPolicy grid_policy,
  CollisionConfig config)
: vehicle_geometry_(vehicle_geometry), grid_policy_(grid_policy), config_(config)
{
}

void CollisionChecker::buildEuclideanTransform(OccupancyGrid & grid) const
{
  if (!gridGeometryValid(grid)) {
    grid.obstacle_distance_m.clear();
    grid.has_euclidean_transform = false;
    return;
  }

  const size_t cell_count = static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height);
  if (grid.data.size() < cell_count) {
    grid.obstacle_distance_m.clear();
    grid.has_euclidean_transform = false;
    return;
  }

  std::vector<double> row_distance_sq(cell_count, kInfDistanceSqCells);
  std::vector<double> distance_sq(cell_count, kInfDistanceSqCells);
  std::vector<double> f(static_cast<size_t>(std::max(grid.width, grid.height)));
  std::vector<double> d;

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const int index = gridIndex(row, col, grid.width);
      f[static_cast<size_t>(col)] = grid_policy_.isOccupied(
        grid.data[static_cast<size_t>(index)]) ? 0.0 : kInfDistanceSqCells;
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
  grid.has_euclidean_transform = true;
}

CollisionCheckResult CollisionChecker::applyOutOfGridPolicy(
  CollisionCheckResult result) const
{
  if (result.status == CollisionStatus::OUT_OF_GRID &&
    grid_policy_.treat_out_of_grid_as_free)
  {
    result.status = CollisionStatus::FREE;
  }
  return result;
}

CollisionCheckResult CollisionChecker::collisionCheck(
  const std::vector<CurveSample> & path,
  const OccupancyGrid & grid) const
{
  if (!euclideanTransformValid(grid) || path.empty()) {
    return applyOutOfGridPolicy(
      {CollisionStatus::OUT_OF_GRID, -std::numeric_limits<double>::infinity(), 0});
  }

  const float * const edt = grid.obstacle_distance_m.data();
  const int width = grid.width;
  const int height = grid.height;
  const double origin_x = grid.origin.x;
  const double origin_y = grid.origin.y;
  const double resolution = grid.resolution;
  const double collision_radius_m = std::max(0.0, vehicle_geometry_.collision_radius_m);
  const double soft_inflation_distance_m = std::max(0.0, config_.soft_inflation_distance_m);
  const double cell_half_diagonal = 0.5 * std::sqrt(2.0) * resolution;
  const double front_offset_m = vehicle_geometry_.front_circle_offset_m;

  const double max_step_m = resolution;

  CollisionStatus aggregated_status = CollisionStatus::FREE;
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  uint32_t checked_poses = 0;

  auto checkCircle = [&](double x, double y) -> CollisionStatus {
      const int col = static_cast<int>(std::floor((x - origin_x) / resolution));
      const int row = static_cast<int>(std::floor((y - origin_y) / resolution));
      if (col < 0 || col >= width || row < 0 || row >= height) {
        return CollisionStatus::OUT_OF_GRID;
      }

      const double clearance_m =
        static_cast<double>(edt[static_cast<size_t>(gridIndex(row, col, width))]) -
        cell_half_diagonal - collision_radius_m;
      minimum_clearance_m = std::min(minimum_clearance_m, clearance_m);
      if (clearance_m <= 0.0) {
        return CollisionStatus::COLLISION;
      }
      if (clearance_m <= soft_inflation_distance_m) {
        return CollisionStatus::SOFT_INFLATION;
      }
      return CollisionStatus::FREE;
    };

  // Rear circle at (x, y), front circle at (x, y) + (front_dx, front_dy).

  auto checkPose = [&](double x, double y, double front_dx, double front_dy) -> bool {
      ++checked_poses;
      const CollisionStatus rear_status = checkCircle(x, y);
      if (rear_status == CollisionStatus::OUT_OF_GRID) {
        aggregated_status = CollisionStatus::OUT_OF_GRID;
        minimum_clearance_m = -std::numeric_limits<double>::infinity();
        return false;
      }
      if (rear_status == CollisionStatus::COLLISION) {
        aggregated_status = CollisionStatus::COLLISION;
        return true;
      }
      aggregated_status = worseStatus(aggregated_status, rear_status);

      const CollisionStatus front_status = checkCircle(x + front_dx, y + front_dy);
      if (front_status == CollisionStatus::OUT_OF_GRID) {
        aggregated_status = CollisionStatus::OUT_OF_GRID;
        minimum_clearance_m = -std::numeric_limits<double>::infinity();
        return false;
      }
      aggregated_status = worseStatus(aggregated_status, front_status);
      return true;
    };

  auto seedFrontOffset = [front_offset_m](double heading, double & front_dx, double & front_dy) {
      front_dx = front_offset_m * std::cos(heading);
      front_dy = front_offset_m * std::sin(heading);
    };

  if (path.size() == 1) {
    double front_dx = 0.0;
    double front_dy = 0.0;
    seedFrontOffset(path.front().heading, front_dx, front_dy);
    checkPose(path.front().x, path.front().y, front_dx, front_dy);
    return applyOutOfGridPolicy({aggregated_status, minimum_clearance_m, checked_poses});
  }

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const CurveSample & a = path[i];
    const CurveSample & b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_length = std::sqrt(dx * dx + dy * dy);
    const double heading_delta = shortestAngleDiff(b.heading, a.heading);
    const int step_count = std::max(
      1,
      static_cast<int>(std::ceil(segment_length / std::max(max_step_m, kEpsilon))));
    const double step_dheading = heading_delta / static_cast<double>(step_count);
    const double cos_d = std::cos(step_dheading);
    const double sin_d = std::sin(step_dheading);

    // Heading is linear along the segment, so rotating the front-circle offset
    // by a constant step is exact. Re-seed each segment to bound drift.
    double front_dx = 0.0;
    double front_dy = 0.0;
    seedFrontOffset(a.heading, front_dx, front_dy);

    for (int step = 0; step <= step_count; ++step) {
      // Skip the shared endpoint already evaluated at the end of the prev
      if (!(i > 0 && step == 0)) {
        const double t = static_cast<double>(step) / static_cast<double>(step_count);
        const double x = a.x + t * dx;
        const double y = a.y + t * dy;
        if (!checkPose(x, y, front_dx, front_dy)) {
          return applyOutOfGridPolicy(
            {aggregated_status, minimum_clearance_m, checked_poses});
        }
      }
      if (step == step_count) {
        break;
      }
      const double next_front_dx = front_dx * cos_d - front_dy * sin_d;
      front_dy = front_dx * sin_d + front_dy * cos_d;
      front_dx = next_front_dx;
    }
  }

  return applyOutOfGridPolicy({aggregated_status, minimum_clearance_m, checked_poses});
}

} // namespace local_planning
