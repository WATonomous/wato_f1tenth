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

LocalPlannerConfig defaultConfig()
{
  LocalPlannerConfig config;
  config.collision_circle_radius_m = 0.20;
  config.front_collision_circle_offset_m = 0.26;
  config.soft_inflation_distance_m = 0.18;
  config.occupied_threshold = 50;
  return config;
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
  const LocalPlannerConfig config = defaultConfig();
  CollisionChecker checker(config);
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0),
    sample(1.0, 1.0, 0.0),
    sample(2.0, 2.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::FREE);
  EXPECT_TRUE(std::isfinite(result.minimum_clearance_m));
  EXPECT_GT(result.minimum_clearance_m, config.soft_inflation_distance_m);
}

TEST(CollisionChecker, SoftInflationPath)
{
  OccupancyGrid grid = freeField();
  const LocalPlannerConfig config = defaultConfig();
  setOccupied(grid, 53, 50);  // center ≈ (0.05, 0.35)
  CollisionChecker checker(config);
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
  EXPECT_LE(result.minimum_clearance_m, config.soft_inflation_distance_m);
}

TEST(CollisionChecker, HardCollisionPath)
{
  OccupancyGrid grid = freeField();
  setOccupied(grid, 50, 60);
  const LocalPlannerConfig config = defaultConfig();
  CollisionChecker checker(config);
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
  const LocalPlannerConfig config = defaultConfig();
  CollisionChecker checker(config);
  checker.buildEuclideanTransform(grid);

  const std::vector<CurveSample> path = {
    sample(0.0, 0.0, 0.0),
    sample(2.0, 2.0, 0.0),
  };

  const CollisionCheckResult result = checker.collisionCheck(path, grid);
  EXPECT_EQ(result.status, CollisionStatus::COLLISION);
}

TEST(CollisionChecker, FrontCircleOnlyCollision)
{
  OccupancyGrid grid = freeField();
  const LocalPlannerConfig config = defaultConfig();
  setOccupied(grid, 50, 52);
  CollisionChecker checker(config);
  checker.buildEuclideanTransform(grid);

  const Point obstacle = cellCenter(grid, 50, 52);
  const Point rear(0.0, 0.0);
  const Point front(config.front_collision_circle_offset_m, 0.0);
  ASSERT_GT(std::hypot(obstacle.x - rear.x, obstacle.y - rear.y), config.collision_circle_radius_m);
  ASSERT_LE(std::hypot(obstacle.x - front.x, obstacle.y - front.y),
      config.collision_circle_radius_m);

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
  const LocalPlannerConfig config = defaultConfig();
  CollisionChecker checker(config);
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
  const LocalPlannerConfig config = defaultConfig();
  CollisionChecker checker(config);
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
  const LocalPlannerConfig config = defaultConfig();
  CollisionChecker checker(config);
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
  LocalPlannerConfig config = defaultConfig();
  config.front_collision_circle_offset_m = 0.0;
  CollisionChecker checker(config);
  checker.buildEuclideanTransform(grid);

  const CollisionCheckResult result = checker.collisionCheck({sample(0.0, 0.0, 0.0)}, grid);
  EXPECT_EQ(result.status, CollisionStatus::COLLISION);
  EXPECT_LE(result.minimum_clearance_m, 0.0);
}

} // namespace local_planning
