#include <gtest/gtest.h>
#include "mpc_controller/model.hpp"

TEST(Model, StraightAndStationary)
{
  auto z = mpc::step({1, 2, 0}, {1, 0}, .5);
  EXPECT_NEAR(z.x, 1.5, 1e-12);
  EXPECT_DOUBLE_EQ(z.y, 2);
  auto stopped = mpc::step(z, {0, .2}, 1);
  EXPECT_DOUBLE_EQ(stopped.x, z.x);
  EXPECT_DOUBLE_EQ(stopped.yaw, z.yaw);
}

TEST(Model, ExactTurnAndSteeringBounds)
{
  const double angle = .2, w = std::tan(angle) / mpc::wheelbase;
  const auto z = mpc::step({}, {1, angle}, 1.2);
  EXPECT_NEAR(z.x, std::sin(w*1.2)/w, 1e-12);
  EXPECT_NEAR(z.y, (1-std::cos(w*1.2))/w, 1e-12);
  EXPECT_NEAR(z.yaw, w*1.2, 1e-12);
  EXPECT_TRUE(mpc::validCommand({1, mpc::steering_min}));
  EXPECT_TRUE(mpc::validCommand({1, mpc::steering_max}));
  EXPECT_FALSE(mpc::validCommand({1, mpc::steering_min - 1e-6}));
  EXPECT_FALSE(mpc::validCommand({1, mpc::steering_max + 1e-6}));
  EXPECT_FALSE(mpc::validCommand({NAN, 0}));
}

TEST(Frame, RoundtripAndFrozenOrigin)
{
  mpc::Pose frame{5, -4, 1.2}, map{7, 3, 1.5};
  const auto local = mpc::toLocal(map, frame);
  const auto restored = mpc::toMap(local, frame);
  EXPECT_NEAR(restored.x, map.x, 1e-12);
  EXPECT_NEAR(restored.y, map.y, 1e-12);
  EXPECT_NEAR(restored.yaw, map.yaw, 1e-12);
  const auto initial = mpc::toLocal(frame, frame);
  EXPECT_DOUBLE_EQ(initial.x, 0);
  EXPECT_DOUBLE_EQ(initial.y, 0);
  EXPECT_DOUBLE_EQ(initial.yaw, 0);
}
