#ifndef PLANNING_PLANNER_EDGE_EVALUATOR_HPP
#define PLANNING_PLANNER_EDGE_EVALUATOR_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/collision_checker.hpp"
#include "planning/types.hpp"

#include <vector>

namespace local_planning
{

struct EdgeEvaluationScratch
{
  std::vector<Point> samples;
  std::vector<double> curvatures;
};

struct EdgeEvaluation
{
  CollisionStatus collision_status = CollisionStatus::FREE;
  double predicted_time_cost = 0.0;
  double curvature_change_cost = 0.0;
  double intent_bias_cost = 0.0;   //how much we pull to raceline
  double obstacle_proximity_cost = 0.0;
  double total_cost = 0.0;
};

class FrenetEdgeEvaluator
{
public:
  FrenetEdgeEvaluator(
    const LocalFrenetPlannerConfig & config,
    const CollisionChecker & collision_checker);

  EdgeEvaluation evaluateEdge(
    double d_start,
    double slope_start,
    double second_derivative_start,
    double d_end,
    double slope_end,
    double second_derivative_end,
    LocalPlannerIntent intent,
    const OccupancyGrid & grid,
    const ReferenceGeometrySample * ref_samples,
    int sample_count,
    EdgeEvaluationScratch & scratch) const;

private:
  const LocalFrenetPlannerConfig & config_;
  const CollisionChecker & collision_checker_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_EDGE_EVALUATOR_HPP
