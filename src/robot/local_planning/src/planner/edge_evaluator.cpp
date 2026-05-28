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
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker)
: config_(config),
  frenet_converter_(frenet_converter),
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
  double s_start,
  double d_start,
  double slope_start,
  double d_end,
  LocalPlannerIntent intent,
  const OccupancyGrid & grid) const
{
  EdgeEvaluation edge;
  const QuinticPolynomial curve = computeQuintic(
    d_start, slope_start, d_end, 0.0, config_.layer_spacing_m);
  const int sample_count =
    std::max(
    2, static_cast<int>(std::ceil(
      config_.layer_spacing_m / config_.sample_spacing_m)) + 1);
  const double max_path_angle_rad = config_.max_path_angle_deg * kPi / 180.0;

  edge.samples.reserve(static_cast<size_t>(sample_count));
  std::vector<double> curvatures(static_cast<size_t>(sample_count), 0.0);
  //get samples along quintic
  for (int i = 0; i < sample_count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(sample_count - 1);
    const double s = s_start + t * config_.layer_spacing_m;
    Point p = frenet_converter_.frenetToCartesian({s, curve.evaluate(t)});

    const double path_slope = curve.evaluateDerivative(t) / curve.delta_s;
    const double path_angle = std::atan(path_slope);
    if (std::abs(path_angle) > max_path_angle_rad) {
      edge.collision_status = CollisionStatus::GEOMETRY_CONSTRAINT;
      return edge;
    }

    const double path_heading = frenet_converter_.getRacingLineHeading(s) + path_angle;
    const CollisionStatus status = collision_checker_.collisionStatus(p, path_heading, grid);
    if (status != CollisionStatus::FREE) {
      edge.collision_status = status;
      return edge;
    }

    edge.samples.push_back(p);
  }
  //get all the curvatures
  for (int i = 1; i + 1 < sample_count; ++i) {
    curvatures[static_cast<size_t>(i)] = computeCurvature(
      edge.samples[static_cast<size_t>(i - 1)],
      edge.samples[static_cast<size_t>(i)],
      edge.samples[static_cast<size_t>(i + 1)]);
  }
  if (sample_count > 2) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[static_cast<size_t>(sample_count - 2)];
  }
  //get velocities across the samples
  for (int i = 0; i < sample_count; ++i) {
    const double s = s_start + (static_cast<double>(i) / static_cast<double>(sample_count - 1)) *
      config_.layer_spacing_m;
    edge.samples[static_cast<size_t>(i)].velocity =
      computeVelocity(s, curvatures[static_cast<size_t>(i)], frenet_converter_, config_);
  }
  /*
  the stuff below is just adding together a bunch of subcosts for the samples
  */
  for (int i = 1; i < sample_count; ++i) {
    const Point & prev = edge.samples[static_cast<size_t>(i - 1)];
    const Point & curr = edge.samples[static_cast<size_t>(i)];
    const double segment_length = distance(prev, curr);
    const double v = std::max(config_.min_velocity_mps, 0.5 * (prev.velocity + curr.velocity));
    edge.predicted_time_cost += segment_length / v;

    const double dk = curvatures[static_cast<size_t>(i)] - curvatures[static_cast<size_t>(i - 1)];
    edge.curvature_change_cost += dk * dk;
  }

  for (const Point & sample : edge.samples) {
    const FrenetPoint fp = frenet_converter_.cartesianToFrenet(sample);
    edge.intent_bias_cost += intentBias(fp.d, intent, config_);
  }
  edge.intent_bias_cost /= static_cast<double>(edge.samples.size());

  edge.total_cost =
    config_.time_weight * edge.predicted_time_cost +
    config_.curvature_change_weight * edge.curvature_change_cost +
    edge.intent_bias_cost;
  return edge;
}

} // namespace local_planning
