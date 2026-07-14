#include "planning/planner/edge_evaluator.hpp"

#include "planning/planner/planner_costs.hpp"
#include "planning/planner/quintic_polynomial.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

} // namespace

FrenetEdgeEvaluator::FrenetEdgeEvaluator(
  const LocalFrenetPlannerConfig & config,
  const CollisionChecker & collision_checker)
: config_(config),
  collision_checker_(collision_checker)
{
}

/*
    make sure the hop is valid and figure out its cost
    1.  build quintic in d over layer spacing
    2.  map it to x,y
    3.  reject if its out of grid space or collides with smth
        TODO:   it would be bad if there are tiny artifacts in the actual lidar
                that would trigger a fake collision
                this should probably be more robust
    4.  get curvature + velocity on samples and use these to get costs
    5. average out weight * d^2 on all the samples
    6. we return the cost and the samples

*/
EdgeEvaluation FrenetEdgeEvaluator::evaluateEdge(
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
  EdgeEvaluationScratch & scratch) const
{
  EdgeEvaluation edge;
  if (ref_samples == nullptr || sample_count < 2) {
    edge.collision_status = CollisionStatus::GEOMETRY_CONSTRAINT;
    return edge;
  }

  const double s0 = ref_samples[0].s;
  const double delta_s = ref_samples[sample_count - 1].s - s0;
  if (delta_s <= 1e-12) {
    edge.collision_status = CollisionStatus::GEOMETRY_CONSTRAINT;
    return edge;
  }

  const QuinticPolynomial curve = computeQuintic(
    d_start, slope_start, second_derivative_start,
    d_end, slope_end, second_derivative_end, delta_s);
  const double max_path_angle_rad = config_.max_path_angle_deg * kPi / 180.0;

  scratch.samples.clear();
  scratch.samples.reserve(static_cast<size_t>(sample_count));
  scratch.curvatures.assign(static_cast<size_t>(sample_count), 0.0);
  std::vector<Point> & samples = scratch.samples;
  std::vector<double> & curvatures = scratch.curvatures;
  //get samples along quintic
  for (int i = 0; i < sample_count; ++i) {
    const ReferenceGeometrySample & ref = ref_samples[i];
    const double t = (ref.s - s0) / delta_s;
    const double d = curve.evaluate(t);
    Point p = FrenetConverter::frenetToCartesian(ref, d);

    const double path_slope = curve.evaluateDerivative(t) / curve.delta_s;
    const double path_angle = std::atan(path_slope);
    if (std::abs(path_angle) > max_path_angle_rad) {
      edge.collision_status = CollisionStatus::GEOMETRY_CONSTRAINT;
      return edge;
    }

    const double path_heading = ref.heading + path_angle;
    const CollisionStatus status = collision_checker_.collisionStatus(p, path_heading, grid);
    if (status == CollisionStatus::SOFT_INFLATION) {
      edge.obstacle_proximity_cost = config_.soft_inflation_cost;
    } else if (status != CollisionStatus::FREE) {
      edge.collision_status = status;
      return edge;
    }

    samples.push_back(p);
    edge.intent_bias_cost += intentBias(d, intent, config_);
  }
  //get all the curvatures
  for (int i = 1; i + 1 < sample_count; ++i) {
    curvatures[static_cast<size_t>(i)] = computeCurvature(
      samples[static_cast<size_t>(i - 1)],
      samples[static_cast<size_t>(i)],
      samples[static_cast<size_t>(i + 1)]);
  }
  if (sample_count > 2) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[static_cast<size_t>(sample_count - 2)];
  }
  //get velocities across the samples
  for (int i = 0; i < sample_count; ++i) {
    samples[static_cast<size_t>(i)].velocity =
      computeVelocity(ref_samples[i].velocity, curvatures[static_cast<size_t>(i)], config_);
  }
  /*
  the stuff below is just adding together a bunch of subcosts for the samples
  */
  for (int i = 1; i < sample_count; ++i) {
    const Point & prev = samples[static_cast<size_t>(i - 1)];
    const Point & curr = samples[static_cast<size_t>(i)];
    const double segment_length = distance(prev, curr);
    const double v = std::max(config_.min_velocity_mps, 0.5 * (prev.velocity + curr.velocity));
    edge.predicted_time_cost += segment_length / v;

    const double dk = curvatures[static_cast<size_t>(i)] - curvatures[static_cast<size_t>(i - 1)];
    edge.curvature_change_cost += dk * dk;
  }

  edge.intent_bias_cost /= static_cast<double>(samples.size());

  edge.total_cost =
    config_.time_weight * edge.predicted_time_cost +
    config_.curvature_change_weight * edge.curvature_change_cost +
    edge.intent_bias_cost +
    edge.obstacle_proximity_cost;
  return edge;
}

} // namespace local_planning
