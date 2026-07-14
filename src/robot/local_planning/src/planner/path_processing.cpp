#include "planning/planner/path_processing.hpp"

#include "planning/planner/quintic_polynomial.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kMinSegmentLengthM = 1e-6;
constexpr double kPi = 3.14159265358979323846;

double maxSlope(const LocalFrenetPlannerConfig & config)
{
  return std::tan(config.max_path_angle_deg * kPi / 180.0);
}

double distanceBetween(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double clampVelocity(double velocity, const LocalFrenetPlannerConfig & config)
{
  return std::min(config.max_velocity_mps, std::max(config.min_velocity_mps, velocity));
}

bool hasHardCollision(
  const Point & point,
  double heading,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid)
{
  const CollisionStatus status = collision_checker.collisionStatus(point, heading, grid);
  return status == CollisionStatus::COLLISION || status == CollisionStatus::OUT_OF_GRID;
}

} // namespace

std::vector<Point> smoothFrenetAnglesOrFallback(
  const std::vector<FrenetPoint> & anchors,
  const std::vector<Point> & fallback_path,
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & config,
  bool & used_smoothed_path)
{
  used_smoothed_path = false;
  if (!config.angle_smoothing_enabled || anchors.size() < 2 ||
    config.sample_spacing_m <= kEpsilon)
  {
    return fallback_path;
  }

  std::vector<FrenetPoint> smoothed_anchors = anchors;
  const double slope_limit = maxSlope(config);
  smoothed_anchors.front().slope = std::clamp(
    smoothed_anchors.front().slope, -slope_limit, slope_limit);
  smoothed_anchors.back().slope = 0.0;

  for (std::size_t i = 1; i + 1 < smoothed_anchors.size(); ++i) {
    const double ds = smoothed_anchors[i + 1].s - smoothed_anchors[i - 1].s;
    if (ds <= kEpsilon) {
      smoothed_anchors[i].slope = 0.0;
      continue;
    }

    const double slope = (smoothed_anchors[i + 1].d - smoothed_anchors[i - 1].d) / ds;
    smoothed_anchors[i].slope = std::clamp(slope, -slope_limit, slope_limit);
  }

  std::vector<Point> path;
  path.reserve(fallback_path.size());
  for (std::size_t anchor_index = 0; anchor_index + 1 < smoothed_anchors.size(); ++anchor_index) {
    const FrenetPoint & start = smoothed_anchors[anchor_index];
    const FrenetPoint & end = smoothed_anchors[anchor_index + 1];
    const double delta_s = end.s - start.s;
    if (delta_s <= kEpsilon) {
      return fallback_path;
    }

    const QuinticPolynomial curve = computeQuintic(
      start.d, start.slope, start.second_derivative,
      end.d, end.slope, end.second_derivative, delta_s);
    const int sample_count = std::max(
      2, static_cast<int>(std::ceil(delta_s / config.sample_spacing_m)) + 1);

    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
      if (!path.empty() && sample_index == 0) {
        continue;
      }

      const double t = static_cast<double>(sample_index) /
        static_cast<double>(sample_count - 1);
      const double s = start.s + t * delta_s;
      const double d = curve.evaluate(t);
      const double path_slope = curve.evaluateDerivative(t) / curve.delta_s;
      const double path_heading = frenet_converter.getRacingLineHeading(s) +
        std::atan(path_slope);
      Point point = frenet_converter.frenetToCartesian({s, d});
      if (hasHardCollision(point, path_heading, collision_checker, grid)) {
        return fallback_path;
      }

      path.push_back(point);
    }
  }

  if (path.empty()) {
    return fallback_path;
  }

  used_smoothed_path = true;
  return path;
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
    point.velocity = clampVelocity(point.velocity, config);
  }
}

} // namespace local_planning
