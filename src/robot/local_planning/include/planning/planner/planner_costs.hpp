#ifndef PLANNING_PLANNER_PLANNER_COSTS_HPP
#define PLANNING_PLANNER_PLANNER_COSTS_HPP

#include "planning/frenet_converter.hpp"
#include "planning/types.hpp"

namespace local_planning
{

double distance(const Point & a, const Point & b);

double computeCurvature(
  const Point & prev,
  const Point & curr,
  const Point & next);

double computeVelocity(
  double s,
  double curvature,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config);

double intentBias(
  double d,
  LocalPlannerIntent intent,
  const LocalFrenetPlannerConfig & config);

double normalizeHeadingError(double angle);

} // namespace local_planning

#endif // PLANNING_PLANNER_PLANNER_COSTS_HPP
