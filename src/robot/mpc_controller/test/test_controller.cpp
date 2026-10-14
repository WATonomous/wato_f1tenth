#include <gtest/gtest.h>
#include "fixtures.hpp"
#include "mpc_controller/controller.hpp"

TEST(Controller, SmallLateralRejoinPreservesInitialHeading)
{
  mpc::Reference reference;circleTrack(reference);
  mpc::Scene scene(freeGrid(),reference,1);
  mpc::Controller controller;
  const auto cycle=controller.cycle({0,.14,0},{.4,0},.02,reference,scene,
    mpc::Clock::now()+std::chrono::seconds(1));
  ASSERT_GT(cycle.prepare_ms,0);
  const auto & problem=cycle.problem;
  ASSERT_GT(problem.targets.back().x,.1);
  EXPECT_LT(std::abs(problem.targets[0].yaw),.1);
  for (int k=1;k<=mpc::horizon;++k) {
    EXPECT_LT(std::abs(mpc::angleDifference(problem.targets[k].yaw,problem.targets[k-1].yaw)),.1);
  }
}

TEST(Controller, SoloCycleAndLatestPose)
{
  mpc::Reference reference; circleTrack(reference);
  mpc::Scene scene(freeGrid(),reference,1);
  mpc::Controller controller;
  mpc::Pose ego;
  mpc::Command previous;
  int approved = 0;
  double largest_error=0;
  for (int i=0; i<500; ++i) {
    auto cycle=controller.cycle(ego,previous,.02,reference,scene,
      mpc::Clock::now()+std::chrono::seconds(1));
    if (cycle.validation.accepted) {
      ++approved;
      EXPECT_DOUBLE_EQ(cycle.problem.frame.x,ego.x);
      EXPECT_DOUBLE_EQ(cycle.problem.frame.y,ego.y);
      EXPECT_NEAR(cycle.solution.states[0][0],0,1e-10);
      previous=cycle.solution.commands[0];
    } else {previous={};}
    ego=mpc::step(ego,previous,.02);
    largest_error=std::max(largest_error,std::abs(std::hypot(ego.x,ego.y-10)-10));
  }
  EXPECT_GT(approved,490);
  EXPECT_LT(largest_error,.1);
  EXPECT_GT(ego.x,.15);
}

TEST(Controller, RejectsExpiredAndBlockedScenes)
{
  mpc::Reference reference; circleTrack(reference);
  auto grid=freeGrid();
  mpc::Scene scene(grid,reference,1);
  mpc::Controller controller;
  auto expired=controller.cycle({}, {}, .02,reference,scene,mpc::Clock::now());
  EXPECT_FALSE(expired.validation.accepted);
  std::fill(grid.cells.begin(),grid.cells.end(),100);
  mpc::Scene blocked(grid,reference,2);
  auto result=controller.cycle({}, {}, .02,reference,blocked,
    mpc::Clock::now()+std::chrono::seconds(1));
  EXPECT_FALSE(result.validation.accepted);
}

TEST(Controller, RejectsCorruptedCandidateAndWrongVersion)
{
  mpc::Reference reference; circleTrack(reference);
  mpc::Scene scene(freeGrid(),reference,1);
  mpc::Controller controller;
  auto cycle=controller.cycle({}, {}, .02,reference,scene,
    mpc::Clock::now()+std::chrono::seconds(1));
  for (int i=0;i<4 && !cycle.validation.accepted;++i) {
    cycle=controller.cycle({}, {}, .02,reference,scene,
      mpc::Clock::now()+std::chrono::seconds(1));
  }
  ASSERT_TRUE(cycle.validation.accepted) << cycle.validation.reason;
  cycle.solution.states[5][1]+=1;
  EXPECT_FALSE(mpc::validate(cycle.problem,cycle.solution,scene,
    mpc::Clock::now()+std::chrono::seconds(1)).accepted);
  cycle.solution.scene_version=999;
  EXPECT_FALSE(mpc::validate(cycle.problem,cycle.solution,scene,
    mpc::Clock::now()+std::chrono::seconds(1)).accepted);
}

TEST(Controller, CompletesOneStaticDetourAndRetiresPassedVertices)
{
  mpc::Reference reference; circleTrack(reference,3.);
  auto grid=freeGrid();
  grid.width=260;grid.height=260;grid.origin={-13,-3,0};
  grid.cells.assign(grid.width*grid.height,0);
  for (int y=0;y<grid.height;++y) {
    for (int x=0;x<grid.width;++x) {
      const double mx=-13+(x+.5)*.1,my=-3+(y+.5)*.1;
      if (mx>2.8 && mx<3.2 && my>-.3 && my<.9) {grid.cells[y*grid.width+x]=100;}
    }
  }
  mpc::Scene scene(grid,reference,1);
  mpc::Controller controller;
  mpc::Pose ego;mpc::Command previous;
  bool passed=false;
  for (int i=0;i<1200;++i) {
    const auto cycle=controller.cycle(ego,previous,.02,reference,scene,
      mpc::Clock::now()+std::chrono::seconds(1));
    previous=cycle.validation.accepted ? cycle.solution.commands[0] : mpc::Command{};
    ego=mpc::step(ego,previous,.02);
    ASSERT_GE(scene.clearanceAt(ego),mpc::clearance);
    if (ego.x>4.5) {passed=true;break;}
  }
  EXPECT_TRUE(passed) << ego.x << "," << ego.y;
}

TEST(Validation, RejectsAnObstacleBetweenClearStageEndpoints)
{
  mpc::Reference reference;circleTrack(reference);
  mpc::Grid grid;
  grid.width=600;grid.height=600;grid.resolution=.01;
  const double edge=mpc::coveringRadius()+mpc::clearance-.00005;
  grid.origin={-2.981+(mpc::front_extent-mpc::rear_extent)/2,edge-3,0};
  grid.cells.assign(600*600,0);
  grid.cells[300*600+300]=100;
  mpc::Scene scene(grid,reference,1);
  ASSERT_GT(scene.clearanceAt({0,0,0}),mpc::clearance);
  ASSERT_GT(scene.clearanceAt({.04,0,0}),mpc::clearance);
  ASSERT_LT(scene.clearanceAt({.02,0,0}),mpc::clearance);
  mpc::Problem problem;mpc::Solution solution;
  problem.scene_version=solution.scene_version=1;
  problem.track_version=solution.track_version=reference.version();
  problem.previous={1,0};problem.first_interval=.02;
  solution.status=0;solution.inequality_residual=solution.dynamics_residual=0;
  mpc::Pose pose;mpc::Command memory=problem.previous;
  for (int k=0;k<=mpc::horizon;++k) {
    problem.corridor[k]={{{1,0,100},{-1,0,100},{0,1,100},{0,-1,100}}};
    solution.states[k]={pose.x,pose.y,pose.yaw,memory.speed,memory.steering};
    if (k<mpc::horizon) {
      memory={std::min(1.,mpc::speed_rate*(mpc::horizon-1-k)*mpc::dt),0};
      solution.commands[k]=memory;pose=mpc::step(pose,memory,mpc::dt);
    }
  }
  const auto result=mpc::validate(problem,solution,scene,mpc::Clock::now()+std::chrono::seconds(1));
  EXPECT_FALSE(result.accepted);EXPECT_EQ(result.reason,"CollisionOrTrack");
}
