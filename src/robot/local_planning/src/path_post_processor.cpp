#include "planning/path_post_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{

constexpr double kMinSegmentLengthM = 1e-6;

double distanceBetween(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double clampVelocity(double velocity, const LocalFrenetPlannerConfig & planner_config)
{
  return std::min(
    planner_config.max_velocity_mps,
    std::max(planner_config.min_velocity_mps, velocity));
}

} // namespace

void PathPostProcessor::setConfig(const PathPostProcessorConfig & config)
{
  config_ = config;
}

void PathPostProcessor::process(
  std::vector<Point> & path,
  const Odometry & odom,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & planner_config) const
{
  (void)grid;

  smoothVelocities(path, odom, planner_config);
}

void PathPostProcessor::smoothVelocities(
  std::vector<Point> & path,
  const Odometry & odom,
  const LocalFrenetPlannerConfig & planner_config) const
{
  if (!config_.velocity_smoothing_enabled || path.size() < 2) {
    return;
  }

  const double max_accel = config_.velocity_smoothing_max_accel_mps2;
  const double max_decel = config_.velocity_smoothing_max_decel_mps2;
  if (max_accel <= 0.0 || max_decel <= 0.0) {
    return;
  }

  for (Point & point : path) {
    point.velocity = clampVelocity(point.velocity, planner_config);
  }

  path.front().velocity = clampVelocity(
    std::min(path.front().velocity, std::max(0.0, odom.velocity)),
    planner_config);

  for (std::size_t i = path.size() - 1; i > 0; --i) {
    const std::size_t prev_index = i - 1;
    const double ds = distanceBetween(path[prev_index], path[i]);
    if (ds <= kMinSegmentLengthM) {
      continue;
    }

    const double next_velocity = path[i].velocity;
    const double allowed_velocity = std::sqrt(
      next_velocity * next_velocity + 2.0 * max_decel * ds);
    path[prev_index].velocity = std::min(path[prev_index].velocity, allowed_velocity);
  }

  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = distanceBetween(path[i - 1], path[i]);
    if (ds <= kMinSegmentLengthM) {
      continue;
    }

    const double prev_velocity = path[i - 1].velocity;
    const double allowed_velocity = std::sqrt(
      prev_velocity * prev_velocity + 2.0 * max_accel * ds);
    path[i].velocity = std::min(path[i].velocity, allowed_velocity);
  }

  for (Point & point : path) {
    point.velocity = clampVelocity(point.velocity, planner_config);
  }
}

} // namespace local_planning
