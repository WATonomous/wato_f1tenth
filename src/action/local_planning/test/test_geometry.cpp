#include "local_planning/core/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace local_planning
{
namespace
{

TEST(Geometry, WrapAngleKeepsValuesInsidePi)
{
  EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
  EXPECT_LE(std::abs(wrapAngle(kPi)), kPi);
  EXPECT_LE(std::abs(wrapAngle(-kPi)), kPi);
  EXPECT_NEAR(wrapAngle(wrapAngle(3.0 * kPi) - wrapAngle(kPi)), 0.0, kSplineEps);
  EXPECT_NEAR(wrapAngle(wrapAngle(-3.0 * kPi) - wrapAngle(-kPi)), 0.0, kSplineEps);
}

TEST(Geometry, WrapAngleDifferenceAcrossTheCutIsShortest)
{
  EXPECT_NEAR(shortestAngleDiff(kPi, -kPi), 0.0, kSplineEps);
  EXPECT_NEAR(shortestAngleDiff(-0.1, 0.1), -0.2, kSplineEps);
  EXPECT_GT(shortestAngleDiff(-kPi + 0.1, kPi - 0.1), 0.0);
  EXPECT_LT(std::abs(shortestAngleDiff(-kPi + 0.1, kPi - 0.1)), 0.3);
}

TEST(Geometry, WrapAnglePropagatesNaN)
{
  const double wrapped = wrapAngle(std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(std::isnan(wrapped));
}

}  // namespace
}  // namespace local_planning
