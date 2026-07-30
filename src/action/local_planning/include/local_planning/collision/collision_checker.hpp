#ifndef LOCAL_PLANNING_COLLISION_COLLISION_CHECKER_HPP
#define LOCAL_PLANNING_COLLISION_COLLISION_CHECKER_HPP

#include "local_planning/core/types.hpp"

namespace local_planning
{

enum class CollisionStatus
{
  FREE,
  SOFT_INFLATION,
  COLLISION,
  OUT_OF_GRID
};

struct CollisionCheckResult
{
  CollisionStatus status = CollisionStatus::FREE;
  double minimum_clearance_m = 0.0;
};

// Dense swept-footprint occupancy checking against the costmap.  Owns nothing
// else: curve construction owns finite-result and curvature checks, the
// sampler owns allowed d ranges, and the velocity module owns dynamic
// feasibility.
class CollisionChecker
{
public:
  explicit CollisionChecker(const LocalPlannerConfig & config);

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
  const LocalPlannerConfig & config_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_COLLISION_COLLISION_CHECKER_HPP
