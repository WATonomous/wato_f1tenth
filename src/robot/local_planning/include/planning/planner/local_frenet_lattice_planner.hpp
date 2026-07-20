#ifndef PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP
#define PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/local_planner.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

class FrenetEdgeEvaluator;

class LocalFrenetLatticePlanner : public LocalPlanner
{
public:
  void setConfig(const LocalFrenetPlannerConfig & config) override;
  void setRacingLine(const std::vector<Point> & racing_line) override;

  LocalFrenetPlan plan(
    const Odometry & odom,
    const OccupancyGrid & grid,
    LocalPlannerIntent intent,
    std::chrono::steady_clock::time_point deadline) override;

private:
  struct DpState
  {
    bool reachable = false;
    double seed_cost = 0.0;
    double predicted_time = 0.0;
    double minimum_clearance_m = 0.0;
    int parent_lane = -1;
    // Cost-to-come is a direct car-to-node quartic rather than a path through
    // the preceding lattice layer.
    bool direct_from_start = false;
  };

  struct SelectedLatticePath
  {
    std::vector<Point> path;
    std::vector<FrenetPoint> anchors;
    int direct_entry_layer = -1;
  };

  std::vector<double> generateLaneOffsets() const;
  int nearestLaneIndex(double d, const std::vector<double> & lanes) const;
  // Shared relaxation for direct quartic source edges and ordinary cubic
  // edges: clearance rank (capped at clearance_cap_m) dominates, then lower
  // seed cost wins.
  static void relaxDpNode(
    DpState & to_state,
    double new_seed_cost,
    double new_predicted_time,
    double new_minimum_clearance_m,
    double clearance_cap_m,
    int parent_lane,
    bool direct_from_start);
  // Copies reference samples for layers [0, destination_layer) into span,
  // dropping the duplicated shared sample at each layer boundary.  Returns the
  // span sample count, or 0 when the table does not cover the request.
  int assembleDirectSpan(
    int destination_layer,
    int sample_count,
    std::vector<ReferenceGeometrySample> & span) const;
  SelectedLatticePath reconstructSelectedPath(
    const std::vector<std::vector<DpState>> & states,
    int final_lane,
    int final_layer,
    const std::vector<double> & lanes,
    const FrenetPoint & start,
    LocalPlannerIntent intent,
    const OccupancyGrid & grid,
    const FrenetEdgeEvaluator & edge_evaluator,
    int sample_count) const;

  FrenetConverter frenet_converter_;
  LocalFrenetPlannerConfig config_;
  std::vector<ReferenceGeometrySample> reference_geometry_table_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_LOCAL_FRENET_LATTICE_PLANNER_HPP
