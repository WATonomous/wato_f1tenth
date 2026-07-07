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
    const Odometry & odom,
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

  struct SelectedLatticePath
  {
    std::vector<Point> path;
    std::vector<FrenetPoint> anchors;
  };

  std::vector<double> generateLaneOffsets() const;
  std::vector<std::vector<Point>> generateDebugLatticeLanes(
    const FrenetPoint & start,
    const std::vector<double> & lanes) const;
  int nearestLaneIndex(double d, const std::vector<double> & lanes) const;
  int selectBestLane(
    const std::vector<std::vector<DpState>> & states,
    int layer,
    const std::vector<double> & lanes,
    LocalPlannerIntent intent) const;
  SelectedLatticePath reconstructSelectedPath(
    const std::vector<std::vector<DpState>> & states,
    int final_layer,
    int final_lane,
    const std::vector<double> & lanes,
    const FrenetPoint & start) const;
  void assignVelocityLimitsFromGeometry(std::vector<Point> & path) const;

  FrenetConverter frenet_converter_;
  LocalFrenetPlannerConfig config_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP
