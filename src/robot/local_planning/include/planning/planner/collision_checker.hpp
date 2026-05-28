#ifndef PLANNING_PLANNER_COLLISION_CHECKER_HPP
#define PLANNING_PLANNER_COLLISION_CHECKER_HPP

#include "planning/types.hpp"

namespace local_planning
{

enum class CollisionStatus
{
  FREE,
  COLLISION,
  OUT_OF_GRID,
  GEOMETRY_CONSTRAINT
};

class CollisionChecker
{
public:
  explicit CollisionChecker(const LocalFrenetPlannerConfig & config);

  CollisionStatus collisionStatus(
    const Point & p,
    double heading,
    const OccupancyGrid & grid) const;

private:
  const LocalFrenetPlannerConfig & config_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_COLLISION_CHECKER_HPP
