#ifndef PLANNING_PLANNER_SPLINE_REFINEMENT_HPP
#define PLANNING_PLANNER_SPLINE_REFINEMENT_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/collision_checker.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

// Refine the selected crude DP path with a Cartesian parametric spline.
//
// The knots are the crude lattice anchors in Cartesian, parameterized by
// cumulative chord length u.  x(u) and y(u) are fit as a quintic first segment
// (clamped to the measured pose, odometry heading, and steering curvature at
// u = 0) joined C2 into a natural C2 cubic spline over the remaining knots.
// The spline is densely sampled, revalidated from scratch (hard collision,
// angle limit, forward progress) and, on success, given curvature-limited
// velocities.
//
// Returns the refined dense path and sets used_spline_path = true on success;
// returns fallback_path unchanged (used_spline_path = false) on any failure.
std::vector<Point> refineWithSplineOrFallback(
  const std::vector<FrenetPoint> & anchors,
  const std::vector<Point> & fallback_path,
  const Odometry & odom,
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & config,
  bool & used_spline_path);

} // namespace local_planning

#endif // PLANNING_PLANNER_SPLINE_REFINEMENT_HPP
