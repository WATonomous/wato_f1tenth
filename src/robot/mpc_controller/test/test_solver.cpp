#include <gtest/gtest.h>
#include "mpc_controller/solver.hpp"

TEST(Solver, FreshPoseBoundedFirstCommandAndTerminalStop)
{
  mpc::Solver solver;
  mpc::Problem p;
  for (int k = 0; k <= mpc::horizon; ++k) {
    p.targets[k] = {k*.02, 0, 0};
    p.corridor[k] = {{{1,0,5}, {-1,0,1}, {0,1,2}, {0,-1,2}}};
    if (k < mpc::horizon) {p.speeds[k] = .5;}
  }
  const auto result = solver.solve(p);
  ASSERT_EQ(result.status, 0);
  EXPECT_NEAR(result.states[0][0], 0, 1e-10);
  EXPECT_LE(result.commands[0].speed, mpc::speed_rate*p.first_interval+1e-7);
  EXPECT_GT(result.commands[0].speed, 0);
  EXPECT_NEAR(result.commands.back().speed, 0, 1e-7);
  EXPECT_LT(result.inequality_residual, 1e-6);
  mpc::Pose z;
  for (int k = 0; k < mpc::horizon; ++k) {
    ASSERT_TRUE(mpc::validCommand(result.commands[k])) << k << ": " <<
      result.commands[k].speed << ", " << result.commands[k].steering;
    EXPECT_DOUBLE_EQ(result.commands[k].speed, static_cast<float>(result.commands[k].speed));
    EXPECT_DOUBLE_EQ(result.commands[k].steering, static_cast<float>(result.commands[k].steering));
    z = mpc::step(z, result.commands[k], mpc::dt);
    EXPECT_NEAR(result.states[k+1][0], z.x, 1e-3);
    EXPECT_NEAR(result.states[k+1][1], z.y, 1e-3);
  }
  p.targets[1].x = NAN;
  EXPECT_EQ(solver.solve(p).status, -1);
}
