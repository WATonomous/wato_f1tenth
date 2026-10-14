#include <gtest/gtest.h>
#include "fixtures.hpp"

TEST(Scene, UnknownBoundaryAndCellEdgesAreBlocked)
{
  mpc::Reference reference;
  circleTrack(reference);
  auto grid = freeGrid();
  grid.cells[30*grid.width+45] = -1;
  mpc::Scene scene(grid, reference, 1);
  EXPECT_TRUE(scene.safe({0,0,0}));
  EXPECT_FALSE(scene.safe({1.5,0,0}));
  EXPECT_LT(scene.clearanceAt({1.49,0,0}), 0);
  EXPECT_FALSE(scene.safe({0,-2,0}));
  EXPECT_FALSE(scene.safe({-10,0,0}));
  grid.cells.pop_back();
  EXPECT_THROW(mpc::Scene invalid(grid, reference, 2), std::invalid_argument);
}

TEST(Scene, RotatedGridPreservesObstacleAndCorridor)
{
  mpc::Reference reference;
  circleTrack(reference);
  auto grid = freeGrid();
  grid.origin = {0,0,.7};
  std::fill(grid.cells.begin(), grid.cells.end(), 0);
  grid.cells[20*grid.width+20] = 100;
  mpc::Scene scene(grid, reference, 1);
  auto occupied = mpc::toMap({2.05,2.05,0}, grid.origin);
  EXPECT_LT(scene.clearanceAt(occupied), 0);
  mpc::Pose frame{1,.5,.3};
  const auto center = mpc::toMap({1.,.8,0}, grid.origin);
  if (scene.safe(center)) {
    const auto faces = scene.rectangle(center, frame);
    const auto local = mpc::toLocal(center, frame);
    for (auto f : faces) {EXPECT_LE(f.x*local.x+f.y*local.y, f.bound);}
  }
}

TEST(Reference, SeamAndInvalidValues)
{
  mpc::Reference reference;
  circleTrack(reference);
  auto a = reference.geometry().sampleAtS(reference.geometry().totalLength()-.01);
  auto b = reference.geometry().sampleAtS(.01);
  EXPECT_LT(std::hypot(a.x-b.x,a.y-b.y), .021);
  EXPECT_GT(a.parameter_speed, .99);
  EXPECT_THROW(reference.set({{NAN,0,1},{1,0,1},{1,1,1},{0,1,1}},
    {{1,1},{1,1},{1,1},{1,1}}), std::invalid_argument);
}
