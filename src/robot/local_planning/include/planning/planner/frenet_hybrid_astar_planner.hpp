#ifndef PLANNING_PLANNER_FRENET_HYBRID_ASTAR_PLANNER_HPP
#define PLANNING_PLANNER_FRENET_HYBRID_ASTAR_PLANNER_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/local_planner.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

class FrenetHybridAStarPlanner : public LocalPlanner
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
  struct SearchState
  {
    int layer_idx = 0;
    int lane_idx = 0;
    int heading_idx = 0;
    double g_cost = 0.0;
    double h_cost = 0.0;
    double f_cost = 0.0;
    double curvature_change_cost = 0.0;
    int parent_index = -1;
    std::vector<Point> edge_samples;
    bool reached = false;
    bool closed = false;
    bool final_candidate_recorded = false;
  };

  struct QueueItem
  {
    double f_cost = 0.0;
    int state_index = -1;
  };

  std::vector<double> generateLaneOffsets() const;
  std::vector<double> generateHeadingBucketsRad() const;
  int nearestLaneIndex(double d, const std::vector<double> & lanes) const;
  int nearestHeadingIndex(double theta_rel, const std::vector<double> & heading_buckets) const;
  int stateIndex(
    int layer_idx,
    int lane_idx,
    int heading_idx,
    int lane_count,
    int heading_count) const;
  double headingToSlope(double theta_rel) const;
  bool headingTransitionFeasible(
    double d_start,
    double d_end,
    double theta_start,
    double theta_end) const;
  bool isBetterPartial(
    const SearchState & candidate,
    const SearchState & incumbent) const;
  double heuristicShotTime(
    int layer_idx,
    double s_start,
    double d,
    double theta_rel,
    LocalPlannerIntent intent,
    int final_layer) const;
  double estimateQuinticShotTime(
    double s_start,
    double remaining_s,
    double d_start,
    double slope_start,
    double d_end,
    double slope_end) const;
  double estimateStraightShotTime(
    double s_start,
    double remaining_s,
    double d_start,
    double slope_start) const;
  double estimateSampledShotTime(
    const std::vector<Point> & samples,
    double s_start,
    double delta_s) const;
  std::vector<Point> reconstructPath(
    const std::vector<SearchState> & states,
    int selected_index) const;

  FrenetConverter frenet_converter_;
  LocalFrenetPlannerConfig config_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_FRENET_HYBRID_ASTAR_PLANNER_HPP
