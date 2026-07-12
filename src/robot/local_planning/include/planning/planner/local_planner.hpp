#ifndef PLANNING_PLANNER_LOCAL_PLANNER_HPP
#define PLANNING_PLANNER_LOCAL_PLANNER_HPP

#include "planning/types.hpp"

#include <vector>
#include <chrono>

namespace local_planning
{

class LocalPlanner
{
public:
  virtual ~LocalPlanner() = default;

  virtual void setConfig(const LocalFrenetPlannerConfig & config) = 0;
  virtual void setRacingLine(const std::vector<Point> & racing_line) = 0;

  virtual LocalFrenetPlan plan(
    const Odometry & odom,
    const OccupancyGrid & grid,
    LocalPlannerIntent intent,
    std::chrono::steady_clock::time_point deadline) = 0;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_LOCAL_PLANNER_HPP
