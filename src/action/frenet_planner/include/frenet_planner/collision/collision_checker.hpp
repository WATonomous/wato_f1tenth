#ifndef FRENET_PLANNER_COLLISION_COLLISION_CHECKER_HPP
#define FRENET_PLANNER_COLLISION_COLLISION_CHECKER_HPP

#include "frenet_planner/core/types.hpp"

#include <vector>

namespace frenet_planner
{

enum class CollisionStatus
{
  FREE,
  SOFT_INFLATION,
  COLLISION,
  OUT_OF_GRID
};

struct CollisionConfig
{
  double soft_inflation_distance_m = 0.18;
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
  CollisionChecker(
    VehicleGeometry vehicle_geometry,
    GridPolicy grid_policy,
    CollisionConfig config);

  // Builds the Euclidean distance transform into obstacle_distance_m.
  // Required before collisionCheck.
  void buildEuclideanTransform(OccupancyGrid & grid) const;


  CollisionCheckResult collisionCheck(
    const std::vector<CurveSample> & path,
    const OccupancyGrid & grid) const;

private:
  CollisionCheckResult applyOutOfGridPolicy(CollisionCheckResult result) const;

  VehicleGeometry vehicle_geometry_;
  GridPolicy grid_policy_;
  CollisionConfig config_;
};

} // namespace frenet_planner

#endif // FRENET_PLANNER_COLLISION_COLLISION_CHECKER_HPP
