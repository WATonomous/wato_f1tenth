#ifndef LOCAL_PLANNING_COLLISION_COLLISION_CHECKER_HPP
#define LOCAL_PLANNING_COLLISION_COLLISION_CHECKER_HPP

#include "local_planning/core/types.hpp"

#include <vector>

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
  // Number of swept-footprint poses actually examined.  This is useful for
  // diagnosing maps/resolutions that make collision checking expensive.
  uint32_t checked_poses = 0;
};

// Dense swept-footprint occupancy checking against a costmap whose Euclidean
// distance transform has already been built via buildEuclideanTransform.
class CollisionChecker
{
public:
  explicit CollisionChecker(const LocalPlannerConfig & config);

  // Builds the Euclidean distance transform into obstacle_distance_m.
  // Required before collisionCheck.
  void buildEuclideanTransform(OccupancyGrid & grid) const;


  CollisionCheckResult collisionCheck(
    const std::vector<CurveSample> & path,
    const OccupancyGrid & grid) const;

private:
  CollisionCheckResult collisionCheckPose(
    const Point & p,
    double heading,
    const OccupancyGrid & grid) const;

  const LocalPlannerConfig & config_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_COLLISION_COLLISION_CHECKER_HPP
