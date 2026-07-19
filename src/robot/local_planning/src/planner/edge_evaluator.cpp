#include "planning/planner/edge_evaluator.hpp"

#include "planning/planner/planner_costs.hpp"
#include "planning/planner/frenet_polynomial.hpp"

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
    1.  take the caller's d(s) polynomial over the ref span
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
  const FrenetPolynomial & curve,
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
  const double delta_s = curve.delta_s;
  if (delta_s <= 1e-12) {
    edge.collision_status = CollisionStatus::GEOMETRY_CONSTRAINT;
    return edge;
  }

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
    const CollisionCheckResult collision =
      collision_checker_.collisionCheck(p, path_heading, grid);
    edge.minimum_clearance_m = std::min(
      edge.minimum_clearance_m, collision.minimum_clearance_m);
    if (collision.status != CollisionStatus::FREE &&
      collision.status != CollisionStatus::SOFT_INFLATION)
    {
      edge.collision_status = collision.status;
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
  }

  edge.intent_bias_cost /= static_cast<double>(samples.size());

  edge.total_cost = edge.predicted_time_cost + edge.intent_bias_cost;
  return edge;
}

} // namespace local_planning
