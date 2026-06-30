#ifndef PLANNING_PATH_POST_PROCESSOR_HPP
#define PLANNING_PATH_POST_PROCESSOR_HPP

#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

class PathPostProcessor
{
public:
  void setConfig(const PathPostProcessorConfig & config);

  void process(
    std::vector<Point> & path,
    const Odometry & odom,
    const OccupancyGrid & grid,
    const LocalFrenetPlannerConfig & planner_config) const;

private:
  void smoothVelocities(
    std::vector<Point> & path,
    const Odometry & odom,
    const LocalFrenetPlannerConfig & planner_config) const;

  PathPostProcessorConfig config_;
};

} // namespace local_planning

#endif // PLANNING_PATH_POST_PROCESSOR_HPP
