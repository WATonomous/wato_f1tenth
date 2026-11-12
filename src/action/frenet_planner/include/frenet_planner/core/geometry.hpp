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

#ifndef FRENET_PLANNER_CORE_GEOMETRY_HPP
#define FRENET_PLANNER_CORE_GEOMETRY_HPP

#include <cmath>

namespace frenet_planner
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kGridEps = 1e-6;
constexpr double kSpacingEps = 1e-9;
constexpr double kSplineEps = 1e-12;

inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double shortestAngleDiff(double a, double b)
{
  return wrapAngle(a - b);
}

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_CORE_GEOMETRY_HPP
