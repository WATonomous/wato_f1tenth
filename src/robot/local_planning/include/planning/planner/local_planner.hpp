#ifndef PLANNING_PLANNER_LOCAL_PLANNER_HPP
#define PLANNING_PLANNER_LOCAL_PLANNER_HPP

#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

class LocalPlanner
{
public:
  virtual ~LocalPlanner() = default;

  virtual void setConfig(const LocalFrenetPlannerConfig & config) = 0;
  virtual void setRacingLine(const std::vector<Point> & racing_line) = 0;

  virtual LocalFrenetPlan plan(
    const Point & start_position,
    double start_heading,
    const OccupancyGrid & grid,
    LocalPlannerIntent intent) = 0;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_LOCAL_PLANNER_HPP
