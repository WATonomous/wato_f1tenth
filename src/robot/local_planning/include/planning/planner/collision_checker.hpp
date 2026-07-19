#ifndef PLANNING_PLANNER_COLLISION_CHECKER_HPP
#define PLANNING_PLANNER_COLLISION_CHECKER_HPP

#include "planning/types.hpp"

namespace local_planning
{

enum class CollisionStatus
{
  FREE,
  SOFT_INFLATION,
  COLLISION,
  OUT_OF_GRID,
  GEOMETRY_CONSTRAINT
};

struct CollisionCheckResult
{
  CollisionStatus status = CollisionStatus::FREE;
  double minimum_clearance_m = 0.0;
};

class CollisionChecker
{
public:
  explicit CollisionChecker(const LocalFrenetPlannerConfig & config);

  void buildClearanceCache(OccupancyGrid & grid) const;

  CollisionCheckResult collisionCheck(
    const Point & p,
    double heading,
    const OccupancyGrid & grid) const;

  CollisionStatus collisionStatus(
    const Point & p,
    double heading,
    const OccupancyGrid & grid) const;

private:
  const LocalFrenetPlannerConfig & config_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_COLLISION_CHECKER_HPP
