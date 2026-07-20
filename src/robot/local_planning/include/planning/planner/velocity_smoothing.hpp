#ifndef PLANNING_PLANNER_VELOCITY_SMOOTHING_HPP
#define PLANNING_PLANNER_VELOCITY_SMOOTHING_HPP

#include "planning/frenet_converter.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

// Assign raceline/curvature-limited velocities via computeVelocity().
// Overload without curvatures recomputes three-point Cartesian curvature
// (with endpoint propagation). Overload with curvatures uses the provided
// values as-is (e.g. spline analytic κ).
void assignVelocityLimits(
  std::vector<Point> & path,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config);

void assignVelocityLimits(
  std::vector<Point> & path,
  const std::vector<double> & curvatures,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config);

// Clamp velocities, pin the start speed to measured odometry, then run a
// backward deceleration pass and a forward acceleration pass.
void smoothVelocityProfile(
  std::vector<Point> & path,
  double start_velocity_mps,
  const LocalFrenetPlannerConfig & config);

} // namespace local_planning

#endif // PLANNING_PLANNER_VELOCITY_SMOOTHING_HPP
