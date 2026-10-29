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
