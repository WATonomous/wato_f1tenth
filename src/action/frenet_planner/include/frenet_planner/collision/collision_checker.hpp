// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FRENET_PLANNER_COLLISION_COLLISION_CHECKER_HPP
#define FRENET_PLANNER_COLLISION_COLLISION_CHECKER_HPP

#include <vector>

#include "frenet_planner/core/types.hpp"

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
  CollisionChecker(VehicleGeometry vehicle_geometry, GridPolicy grid_policy, CollisionConfig config);

  // Builds the Euclidean distance transform into obstacle_distance_m.
  // Required before collisionCheck.
  void buildEuclideanTransform(OccupancyGrid & grid) const;

  CollisionCheckResult collisionCheck(const std::vector<CurveSample> & path, const OccupancyGrid & grid) const;

private:
  CollisionCheckResult applyOutOfGridPolicy(CollisionCheckResult result) const;

  VehicleGeometry vehicle_geometry_;
  GridPolicy grid_policy_;
  CollisionConfig config_;
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_COLLISION_COLLISION_CHECKER_HPP
