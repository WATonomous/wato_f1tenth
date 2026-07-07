#ifndef PLANNING_PLANNER_PATH_PROCESSING_HPP
#define PLANNING_PLANNER_PATH_PROCESSING_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/collision_checker.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

std::vector<Point> smoothFrenetAnglesOrFallback(
  const std::vector<FrenetPoint> & anchors,
  const std::vector<Point> & fallback_path,
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & config,
  bool & used_smoothed_path);

void smoothVelocityProfile(
  std::vector<Point> & path,
  double start_velocity_mps,
  const LocalFrenetPlannerConfig & config);

} // namespace local_planning

#endif // PLANNING_PLANNER_PATH_PROCESSING_HPP
