#include "planning/planner/planner_costs.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace local_planning
{
namespace
{

constexpr double kGravity = 9.81;
constexpr double kEpsilon = 1e-6;

} // namespace

std::string intentToString(LocalPlannerIntent intent)
{
  switch (intent) {
    case LocalPlannerIntent::FOLLOW_RACING_LINE:
      return "FOLLOW_RACING_LINE";
    case LocalPlannerIntent::OVERTAKE:
      return "OVERTAKE";
    case LocalPlannerIntent::MERGE:
      return "MERGE";
  }
  return "UNKNOWN";
} //just for logging

double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

/*
    implements 3 point curvature formula basically find k = 1/R where R is the raidus of a circle through
    the 3 points
    the sign matters
    positive = left
*/
double computeCurvature(const Point & prev, const Point & curr, const Point & next)
{
  const double a = distance(curr, next);
  const double b = distance(prev, next);
  const double c = distance(prev, curr);
  const double denom = a * b * c;
  if (denom < kEpsilon) {
    return 0.0;
  }

  const double cross =
    (curr.x - prev.x) * (next.y - prev.y) -
    (curr.y - prev.y) * (next.x - prev.x);
  return 2.0 * cross / denom;
}

//take min of raceline and physics limited, then clamp it between min and max from yaml
double computeVelocity(
  double s,
  double curvature,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config)
{
  double velocity = std::max(config.min_velocity_mps, frenet_converter.getRacingLineVelocity(s));
  if (std::abs(curvature) > kEpsilon) {
    velocity =
      std::min(velocity, std::sqrt(config.friction_coeff * kGravity / std::abs(curvature)));
  }
  if (config.max_velocity_mps > config.min_velocity_mps) {
    velocity = std::min(velocity, config.max_velocity_mps);
  }
  return std::max(config.min_velocity_mps, velocity);
}

//basically depending on the mode we punish the deviation from the racing line accordingly
double intentBias(double d, LocalPlannerIntent intent, const LocalFrenetPlannerConfig & config)
{
  switch (intent) {
    case LocalPlannerIntent::FOLLOW_RACING_LINE:
      return config.follow_d_weight * d * d;
    case LocalPlannerIntent::OVERTAKE:
      return config.overtake_d_weight * d * d;
    case LocalPlannerIntent::MERGE:
      return config.merge_d_weight * d * d;
  }
  return config.follow_d_weight * d * d;
}

//maps it to -pi to pi
//this is a quirky little trick
double normalizeHeadingError(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace local_planning
