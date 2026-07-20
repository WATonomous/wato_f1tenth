#ifndef PLANNING_PLANNER_FRENET_REFINEMENT_HPP
#define PLANNING_PLANNER_FRENET_REFINEMENT_HPP

#include "planning/frenet_converter.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

// Geometry produced by rebuilding Frenet polynomial segments between anchors.
// Headings are parallel to path and are used by shared post-search validation.
struct FrenetRefinementResult
{
  std::vector<Point> path;
  std::vector<double> headings;
  bool success = false;
};

// Rebuild selected-path geometry from Frenet anchors: central-difference
// interior slopes, quartic first segment, cubic later segments, dense sampling.
// Does not validate collisions or assign velocities; callers own fallback.
FrenetRefinementResult rebuildFrenetGeometry(
  const std::vector<FrenetPoint> & anchors,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config);

} // namespace local_planning

#endif // PLANNING_PLANNER_FRENET_REFINEMENT_HPP
