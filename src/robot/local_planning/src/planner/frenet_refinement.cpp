#include "planning/planner/frenet_refinement.hpp"

#include "planning/planner/frenet_polynomial.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1e-6;
constexpr double kPi = 3.14159265358979323846;

double maxSlope(const LocalFrenetPlannerConfig & config)
{
  return std::tan(config.max_path_angle_deg * kPi / 180.0);
}

} // namespace

FrenetRefinementResult rebuildFrenetGeometry(
  const std::vector<FrenetPoint> & anchors,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config)
{
  FrenetRefinementResult result;
  if (anchors.size() < 2 || config.sample_spacing_m <= kEpsilon) {
    return result;
  }

  std::vector<FrenetPoint> rebuilt_anchors = anchors;
  const double slope_limit = maxSlope(config);
  rebuilt_anchors.front().slope = std::clamp(
    rebuilt_anchors.front().slope, -slope_limit, slope_limit);
  rebuilt_anchors.back().slope = 0.0;

  for (std::size_t i = 1; i + 1 < rebuilt_anchors.size(); ++i) {
    const double ds = rebuilt_anchors[i + 1].s - rebuilt_anchors[i - 1].s;
    if (ds <= kEpsilon) {
      rebuilt_anchors[i].slope = 0.0;
      continue;
    }

    const double slope = (rebuilt_anchors[i + 1].d - rebuilt_anchors[i - 1].d) / ds;
    rebuilt_anchors[i].slope = std::clamp(slope, -slope_limit, slope_limit);
  }

  result.path.reserve(rebuilt_anchors.size() * 8);
  result.headings.reserve(rebuilt_anchors.size() * 8);
  for (std::size_t anchor_index = 0; anchor_index + 1 < rebuilt_anchors.size(); ++anchor_index) {
    const FrenetPoint & start = rebuilt_anchors[anchor_index];
    const FrenetPoint & end = rebuilt_anchors[anchor_index + 1];
    const double delta_s = end.s - start.s;
    if (delta_s <= kEpsilon) {
      result.path.clear();
      result.headings.clear();
      return result;
    }

    // same edge model as the search: the measured-start segment is a quartic
    // (terminal d'' free), everything after is a cubic with no d'' constraints
    const FrenetPolynomial curve = (anchor_index == 0) ?
      computeQuartic(
      start.d, start.slope, start.second_derivative,
      end.d, end.slope, delta_s) :
      computeCubic(start.d, start.slope, end.d, end.slope, delta_s);
    const int sample_count = std::max(
      2, static_cast<int>(std::ceil(delta_s / config.sample_spacing_m)) + 1);

    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
      if (!result.path.empty() && sample_index == 0) {
        continue;
      }

      const double t = static_cast<double>(sample_index) /
        static_cast<double>(sample_count - 1);
      const double s = start.s + t * delta_s;
      const double d = curve.evaluate(t);
      const double path_slope = curve.evaluateDerivative(t) / curve.delta_s;
      const double path_heading = frenet_converter.getRacingLineHeading(s) +
        std::atan(path_slope);
      result.path.push_back(frenet_converter.frenetToCartesian({s, d}));
      result.headings.push_back(path_heading);
    }
  }

  if (result.path.empty()) {
    return result;
  }

  result.success = true;
  return result;
}

} // namespace local_planning
