#include "local_planning/speed/velocity_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{

constexpr double kMinSegmentLengthM = 1e-6;

double segmentLength(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double clampVelocity(double velocity, const LocalPlannerConfig & config)
{
  return std::min(config.max_velocity_mps, std::max(config.min_velocity_mps, velocity));
}

} // namespace

void smoothVelocityProfile(
  std::vector<Point> & path,
  double start_velocity_mps,
  const LocalPlannerConfig & config)
{
  if (path.size() < 2) {
    return;
  }

  const double max_accel = config.max_accel_mps2;
  const double max_decel = config.max_decel_mps2;
  if (max_accel <= 0.0 || max_decel <= 0.0) {
    return;
  }

  for (Point & point : path) {
    point.velocity = clampVelocity(point.velocity, config);
  }

  path.front().velocity = clampVelocity(
    std::min(path.front().velocity, std::max(0.0, start_velocity_mps)), config);

  for (std::size_t i = path.size() - 1; i > 0; --i) {
    const std::size_t prev_index = i - 1;
    const double ds = segmentLength(path[prev_index], path[i]);
    if (ds <= kMinSegmentLengthM) {
      continue;
    }

    const double next_velocity = path[i].velocity;
    const double allowed_velocity = std::sqrt(
      next_velocity * next_velocity + 2.0 * max_decel * ds);
    path[prev_index].velocity = std::min(path[prev_index].velocity, allowed_velocity);
  }

  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = segmentLength(path[i - 1], path[i]);
    if (ds <= kMinSegmentLengthM) {
      continue;
    }

    const double prev_velocity = path[i - 1].velocity;
    const double allowed_velocity = std::sqrt(
      prev_velocity * prev_velocity + 2.0 * max_accel * ds);
    path[i].velocity = std::min(path[i].velocity, allowed_velocity);
  }

  for (Point & point : path) {
    point.velocity = clampVelocity(point.velocity, config);
  }
}

} // namespace local_planning
