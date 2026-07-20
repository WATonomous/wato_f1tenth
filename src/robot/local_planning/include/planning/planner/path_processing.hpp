#ifndef PLANNING_PLANNER_PATH_PROCESSING_HPP
#define PLANNING_PLANNER_PATH_PROCESSING_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/collision_checker.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

// Post-search pipeline result: final path plus whether the requested refiner
// succeeded or the reconstructed crude path was used as fallback.
struct PathProcessingResult
{
  std::vector<Point> path;
  bool refinement_succeeded = false;
  bool used_crude_fallback = false;
};

// Single post-search entry point after DP reconstruction:
// selected geometry refinement → validate or fall back to crude →
// recompute curvature / velocity limits → accel/decel smoothing.
PathProcessingResult processSelectedPath(
  const std::vector<Point> & crude_path,
  const std::vector<FrenetPoint> & anchors,
  const Odometry & odom,
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & config);

} // namespace local_planning

#endif // PLANNING_PLANNER_PATH_PROCESSING_HPP
