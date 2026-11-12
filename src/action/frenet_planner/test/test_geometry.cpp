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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "frenet_planner/core/geometry.hpp"

namespace frenet_planner
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
}  // namespace frenet_planner
