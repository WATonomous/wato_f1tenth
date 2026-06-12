#ifndef PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP
#define PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/local_planner.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

class LocalFrenetLatticePlanner : public LocalPlanner
{
public:
  void setConfig(const LocalFrenetPlannerConfig & config) override;
  void setRacingLine(const std::vector<Point> & racing_line) override;

  LocalFrenetPlan plan(
    const Point & start_position,
    double start_heading,
    const OccupancyGrid & grid,
    LocalPlannerIntent intent) override;

private:
  struct DpState
  {
    bool reachable = false;
    double total_cost = 0.0;
    double curvature_change_cost = 0.0;
    int parent_lane = -1;
    std::vector<Point> edge_samples;     //x,y points along the corresponding edge
  };

  std::vector<double> generateLaneOffsets() const;
  int nearestLaneIndex(double d, const std::vector<double> & lanes) const;
  std::vector<Point> reconstructPath(
    const std::vector<std::vector<DpState>> & states,
    int final_lane) const;

  FrenetConverter frenet_converter_;
  LocalFrenetPlannerConfig config_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP
