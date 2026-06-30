#include "planning/path_post_processor.hpp"

namespace local_planning
{

void PathPostProcessor::setConfig(const PathPostProcessorConfig & config)
{
  config_ = config;
}

void PathPostProcessor::process(
  std::vector<Point> & path,
  const Odometry & odom,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & planner_config) const
{
  (void)path;
  (void)odom;
  (void)grid;
  (void)planner_config;
  (void)config_;
}

} // namespace local_planning
