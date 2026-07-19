#ifndef PLANNING_PLANNER_EDGE_EVALUATOR_HPP
#define PLANNING_PLANNER_EDGE_EVALUATOR_HPP

#include "planning/frenet_converter.hpp"
#include "planning/planner/collision_checker.hpp"
#include "planning/planner/frenet_polynomial.hpp"
#include "planning/types.hpp"
#include <limits>

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
  double intent_bias_cost = 0.0;   //how much we pull to raceline
  double minimum_clearance_m = std::numeric_limits<double>::infinity();
  double total_cost = 0.0;
};

class FrenetEdgeEvaluator
{
public:
  FrenetEdgeEvaluator(
    const LocalFrenetPlannerConfig & config,
    const CollisionChecker & collision_checker);

  // The caller chooses the polynomial (quartic source edge, cubic lattice
  // edge, or the smoother's quintic); curve.delta_s must match the ref span.
  EdgeEvaluation evaluateEdge(
    const FrenetPolynomial & curve,
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
