#ifndef PLANNING_PLANNER_SPLINE_REFINEMENT_HPP
#define PLANNING_PLANNER_SPLINE_REFINEMENT_HPP

#include "planning/frenet_converter.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

// Geometry produced by Cartesian spline fitting. Headings and analytic
// curvatures are parallel to path (from parametric derivatives): headings for
// shared validation, curvatures for velocity limits on success.
struct SplineRefinementResult
{
  std::vector<Point> path;
  std::vector<double> headings;
  std::vector<double> curvatures;
  bool success = false;
};

// Fit a Cartesian parametric spline through the crude lattice anchors:
// knot cleanup, quintic-start + C2 natural cubic, dense sampling, and the
// spline-only forward-progress/cusp guard. Does not run hard-collision or
// heading-limit validation and does not assign velocities.
SplineRefinementResult refineSplineGeometry(
  const std::vector<FrenetPoint> & anchors,
  const Odometry & odom,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config);

} // namespace local_planning

#endif // PLANNING_PLANNER_SPLINE_REFINEMENT_HPP
