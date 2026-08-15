#include "local_planning/collision/collision_checker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace local_planning
{
namespace
{

OccupancyGrid makeGrid(int width, int height, double resolution, double origin_x, double origin_y)
{
  OccupancyGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  grid.origin = Point(origin_x, origin_y);
  grid.data.assign(static_cast<size_t>(width * height), 0);
  return grid;
}

void setOccupied(OccupancyGrid & grid, int row, int col, int8_t value = 100)
{
  grid.data[static_cast<size_t>(row * grid.width + col)] = value;
}

Point cellCenter(const OccupancyGrid & grid, int row, int col)
{
  return Point(
    grid.origin.x + (static_cast<double>(col) + 0.5) * grid.resolution,
    grid.origin.y + (static_cast<double>(row) + 0.5) * grid.resolution);
}

CurveSample sample(double s, double x, double y, double heading = 0.0)
{
  CurveSample out;
  out.s = s;
  out.x = x;
  out.y = y;
  out.heading = heading;
  out.curvature = 0.0;
  out.speed = 0.0;
  return out;
}

struct TestPolicies
{
  VehicleGeometry vehicle;
  GridPolicy grid;
  CollisionConfig collision;
};

CollisionChecker makeChecker(const TestPolicies & policies = TestPolicies{})
{
  return CollisionChecker(policies.vehicle, policies.grid, policies.collision);
}

OccupancyGrid freeField()
{
  return makeGrid(100, 100, 0.10, -5.0, -5.0);
}

} // namespace

TEST(CollisionChecker, FreePath)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 90);  // ≈ (4.05, 0.05)
  const TestPolicies policies;
  CollisionChecker checker = makeChecker(policies);
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0),
    sample(1.0, 1.0, 0.0),
    sample(2.0, 2.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::FREE);
  EXPECT_TRUE(std::isfinite(result.minimum_clearance_m));
  EXPECT_GT(result.minimum_clearance_m, policies.collision.soft_inflation_distance_m);
}

TEST(CollisionChecker, SoftInflationPath)
{
  OccupancyGrid grid = freeField();
  const TestPolicies policies;
  setOccupied(grid, 53, 50);  // center ≈ (0.05, 0.35)
  CollisionChecker checker = makeChecker(policies);
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0),
    sample(0.5, 0.5, 0.0),
  };

  const Point obstacle = cellCenter(grid, 53, 50);
  ASSERT_NEAR(std::hypot(obstacle.x, obstacle.y), 0.35, 0.05);

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::SOFT_INFLATION);
  EXPECT_GT(result.minimum_clearance_m, 0.0);
  EXPECT_LE(result.minimum_clearance_m, policies.collision.soft_inflation_distance_m);
}

TEST(CollisionChecker, HardCollisionPath)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 60);
  CollisionChecker checker = makeChecker();
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0),
    sample(1.0, 1.0, 0.0),
    sample(2.0, 2.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::COLLISION);
  EXPECT_LE(result.minimum_clearance_m, 0.0);
}

TEST(CollisionChecker, InterpolationPreventsTunneling)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 60);
  CollisionChecker checker = makeChecker();
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0),
    sample(2.0, 2.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::COLLISION);
}

TEST(CollisionChecker, PathSamplesAtGridResolutionAreNotDensified)
{
  OccupancyGrid grid = makeGrid(20, 20, 0.10, 0.0, 0.0);
  CollisionChecker checker = makeChecker();
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.5, 0.5),
    sample(0.1, 0.6, 0.5),
    sample(0.2, 0.7, 0.5),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::FREE);
  EXPECT_EQ(result.checked_poses, path.size());
}

TEST(CollisionChecker, FrontCircleOnlyCollision)
{
  OccupancyGrid grid = freeField();
  const TestPolicies policies;
  setOccupied(grid, 50, 52);
  CollisionChecker checker = makeChecker(policies);
  checker.buildEuclideanTransform(grid);

  const Point obstacle = cellCenter(grid, 50, 52);
  const Point rear(0.0, 0.0);
  const Point front(policies.vehicle.front_circle_offset_m, 0.0);
  ASSERT_GT(
    std::hypot(obstacle.x - rear.x, obstacle.y - rear.y),
    policies.vehicle.collision_radius_m);
  ASSERT_LE(std::hypot(obstacle.x - front.x, obstacle.y - front.y),
      policies.vehicle.collision_radius_m);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::COLLISION);
}

TEST(CollisionChecker, MinimumClearanceAggregatesOverPath)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 80);
  CollisionChecker checker = makeChecker();
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> far_only = {
    sample(0.0, 0.0, 0.0),
    sample(0.5, 0.5, 0.0),
  };
  const std::vector<CurveSample> nearer = {
    sample(0.0, 0.0, 0.0),
    sample(0.5, 0.5, 0.0),
    sample(1.5, 1.5, 0.0),
  };

  const CollisionCheckResult far_result = checker.collisionCheck(far_only, grid);
  const CollisionCheckResult near_result = checker.collisionCheck(nearer, grid);
  EXPECT_EQ(far_result.status, CollisionStatus::FREE);
  EXPECT_EQ(near_result.status, CollisionStatus::FREE);
  EXPECT_LT(near_result.minimum_clearance_m, far_result.minimum_clearance_m);
}

TEST(CollisionChecker, FullFootprintGridDepartureIsOutOfGrid)
{
  OccupancyGrid grid = makeGrid(20, 20, 0.10, 0.0, 0.0);
  CollisionChecker checker = makeChecker();
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 1.70, 1.0, 0.0),
    sample(0.2, 1.90, 1.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::OUT_OF_GRID);
  EXPECT_TRUE(std::isinf(result.minimum_clearance_m));
  EXPECT_LT(result.minimum_clearance_m, 0.0);
}

TEST(CollisionChecker, MissingEuclideanTransformOrInvalidGridIsOutOfGrid)
{
  CollisionChecker checker = makeChecker();
  const std::vector<CurveSample> path = {sample(0.0, 0.0, 0.0)};

  OccupancyGrid invalid;
  EXPECT_EQ(checker.collisionCheck(path, invalid).status, CollisionStatus::OUT_OF_GRID);

  OccupancyGrid grid = makeGrid(10, 10, 0.10, 0.0, 0.0);
  // Valid geometry, but no Euclidean transform yet.
  EXPECT_EQ(checker.collisionCheck(path, grid).status, CollisionStatus::OUT_OF_GRID);

  grid.data.pop_back();
  EXPECT_EQ(checker.collisionCheck(path, grid).status, CollisionStatus::OUT_OF_GRID);

  grid.data.push_back(0);
  checker.buildEuclideanTransform(grid);
  EXPECT_EQ(
    checker.collisionCheck(std::vector<CurveSample>{}, grid).status,
    CollisionStatus::OUT_OF_GRID);
}

TEST(CollisionChecker, CollisionNearOccupiedCellCorner)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 52);
  TestPolicies policies;
  policies.vehicle.front_circle_offset_m = 0.0;
  CollisionChecker checker = makeChecker(policies);
  checker.buildEuclideanTransform(grid);

  const CollisionCheckResult result = checker.collisionCheck({sample(0.0, 0.0, 0.0)}, grid);
  EXPECT_EQ(result.status, CollisionStatus::COLLISION);
  EXPECT_LE(result.minimum_clearance_m, 0.0);
}

TEST(CollisionChecker, OccupiedThresholdComesFromGridPolicy)
{
  OccupancyGrid strict_grid = freeField();
  setOccupied(strict_grid, 50, 50, 60);
  OccupancyGrid permissive_grid = strict_grid;

  TestPolicies strict_policies;
  strict_policies.grid.occupied_threshold = 75;
  CollisionChecker strict_checker = makeChecker(strict_policies);
  strict_checker.buildEuclideanTransform(strict_grid);
  EXPECT_EQ(
    strict_checker.collisionCheck({sample(0.0, 0.05, 0.05)}, strict_grid).status,
    CollisionStatus::FREE);

  TestPolicies permissive_policies;
  permissive_policies.grid.occupied_threshold = 50;
  CollisionChecker permissive_checker = makeChecker(permissive_policies);
  permissive_checker.buildEuclideanTransform(permissive_grid);
  EXPECT_EQ(
    permissive_checker.collisionCheck({sample(0.0, 0.05, 0.05)}, permissive_grid).status,
    CollisionStatus::COLLISION);
}

TEST(CollisionChecker, UnknownCellsRemainFreeByDefault)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 50, -1);
  CollisionChecker checker = makeChecker();
  checker.buildEuclideanTransform(grid);

  EXPECT_EQ(
    checker.collisionCheck({sample(0.0, 0.05, 0.05)}, grid).status,
    CollisionStatus::FREE);
}

TEST(CollisionChecker, OutOfGridCanRemainFree)
{
  OccupancyGrid grid = makeGrid(20, 20, 0.10, 0.0, 0.0);
  TestPolicies policies;
  policies.grid.treat_out_of_grid_as_free = true;
  CollisionChecker checker = makeChecker(policies);
  checker.buildEuclideanTransform(grid);

  const CollisionCheckResult result = checker.collisionCheck(
    {sample(0.0, 1.70, 1.0), sample(0.2, 1.90, 1.0)}, grid);
  EXPECT_EQ(result.status, CollisionStatus::FREE);
  EXPECT_TRUE(std::isinf(result.minimum_clearance_m));
  EXPECT_LT(result.minimum_clearance_m, 0.0);
}

} // namespace local_planning
