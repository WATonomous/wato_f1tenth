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
    const auto local = mpc::toLocal(mpc::coveringCenter(center), frame);
    for (auto f : faces) {EXPECT_LE(f.x*local.x+f.y*local.y, f.bound);}
  }
}

TEST(Scene, ContinuousCertificateRecoversSafePosesRejectedByFullCellInflation)
{
  mpc::Reference reference;circleTrack(reference);
  auto grid=freeGrid();
  for (int y=34;y<grid.height;++y) {
    for (int x=0;x<grid.width;++x) {grid.cells[y*grid.width+x]=100;}
  }
  mpc::Scene scene(grid,reference,1);
  const mpc::Pose rear{};
  ASSERT_GT(scene.clearanceAt(rear),mpc::clearance+.005);
  ASSERT_TRUE(scene.safe(rear));
  const auto faces=scene.rectangle(rear,{});
  const auto center=mpc::coveringCenter(rear);
  for (auto face:faces) {EXPECT_LT(face.x*center.x+face.y*center.y,face.bound);}
  // The whole certificate box, not just its center, preserves checked clearance.
  const double dx=faces[0].bound-center.x,dy=faces[2].bound-center.y;
  for (double x:{-dx,dx}) {
    for (double y:{-dy,dy}) {
      EXPECT_GE(scene.clearanceAt({x,y,0}),mpc::clearance+.005-1e-10);
    }
  }
  EXPECT_FALSE(scene.safe({0,.1,0}));
  EXPECT_THROW(scene.rectangle({0,.1,0},{}),std::invalid_argument);
}

TEST(Reference, MeasuredLoopsAreValidatedAndOverrideWidthOffsets)
{
  std::vector<local_planning::Point> points{{-2,-2,1},{2,-2,1},{2,2,1},{-2,2,1}};
  std::vector<local_planning::TrackWidth> widths(4,{2,2});
  std::array<std::vector<mpc::Pose>,2> loops{{
    {{-4,-4,0},{4,-4,0},{4,4,0},{-4,4,0}},
    {{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0}}}};
  mpc::Reference reference;
  reference.set(points,widths,loops);
  EXPECT_GT(reference.trackClearance({2,0,0}),0);
  EXPECT_LT(reference.trackClearance({0,0,0}),0);
  EXPECT_LT(reference.trackClearance({5,0,0}),0);
  const auto version=reference.version();
  std::swap(loops[0][1],loops[0][2]);
  EXPECT_THROW(reference.set(points,widths,loops),std::invalid_argument);
  EXPECT_EQ(reference.version(),version);
  loops[0].clear();
  EXPECT_THROW(reference.set(points,widths,loops),std::invalid_argument);
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
