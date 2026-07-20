#include "planning/planner/velocity_smoothing.hpp"

#include "planning/planner/planner_costs.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{

constexpr double kMinSegmentLengthM = 1e-6;

double clampVelocity(double velocity, const LocalFrenetPlannerConfig & config)
{
  return std::min(config.max_velocity_mps, std::max(config.min_velocity_mps, velocity));
}

} // namespace

void assignVelocityLimits(
  std::vector<Point> & path,
  const std::vector<double> & curvatures,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config)
{
  if (path.empty() || curvatures.size() != path.size()) {
    return;
  }

  for (std::size_t i = 0; i < path.size(); ++i) {
    const double s = frenet_converter.cartesianToFrenet(path[i]).s;
    path[i].velocity = computeVelocity(s, curvatures[i], frenet_converter, config);
  }
}

void assignVelocityLimits(
  std::vector<Point> & path,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config)
{
  if (path.empty()) {
    return;
  }

  std::vector<double> curvatures(path.size(), 0.0);
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    curvatures[i] = computeCurvature(path[i - 1], path[i], path[i + 1]);
  }
  if (path.size() > 2) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[path.size() - 2];
  }

  assignVelocityLimits(path, curvatures, frenet_converter, config);
}

void smoothVelocityProfile(
  std::vector<Point> & path,
  double start_velocity_mps,
  const LocalFrenetPlannerConfig & config)
{
  if (!config.velocity_smoothing_enabled || path.size() < 2) {
    return;
  }

  const double max_accel = config.velocity_smoothing_max_accel_mps2;
  const double max_decel = config.velocity_smoothing_max_decel_mps2;
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
    const double ds = distance(path[prev_index], path[i]);
    if (ds <= kMinSegmentLengthM) {
      continue;
    }

    const double next_velocity = path[i].velocity;
    const double allowed_velocity = std::sqrt(
      next_velocity * next_velocity + 2.0 * max_decel * ds);
    path[prev_index].velocity = std::min(path[prev_index].velocity, allowed_velocity);
  }

  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = distance(path[i - 1], path[i]);
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
