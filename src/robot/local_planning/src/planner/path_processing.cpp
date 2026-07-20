#include "planning/planner/path_processing.hpp"

#include "planning/planner/frenet_refinement.hpp"
#include "planning/planner/planner_costs.hpp"
#include "planning/planner/spline_refinement.hpp"
#include "planning/planner/velocity_smoothing.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

bool hasHardCollision(
  const Point & point,
  double heading,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid)
{
  const CollisionStatus status = collision_checker.collisionStatus(point, heading, grid);
  return status == CollisionStatus::COLLISION || status == CollisionStatus::OUT_OF_GRID;
}

// Shared hard-collision and heading-limit checks for refined geometry.
bool validateRefinedGeometry(
  const std::vector<Point> & path,
  const std::vector<double> & headings,
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & config)
{
  if (path.empty() || path.size() != headings.size()) {
    return false;
  }

  const double max_heading_error = config.max_path_angle_deg * kPi / 180.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (hasHardCollision(path[i], headings[i], collision_checker, grid)) {
      return false;
    }

    const FrenetPoint fp = frenet_converter.cartesianToFrenet(path[i]);
    const double heading_error = normalizeHeadingError(
      headings[i] - frenet_converter.getRacingLineHeading(fp.s));
    if (std::abs(heading_error) > max_heading_error) {
      return false;
    }
  }
  return true;
}

void applyVelocityPipeline(
  std::vector<Point> & path,
  double start_velocity_mps,
  const FrenetConverter & frenet_converter,
  const LocalFrenetPlannerConfig & config,
  const std::vector<double> * analytic_curvatures = nullptr)
{
  if (analytic_curvatures != nullptr &&
    analytic_curvatures->size() == path.size())
  {
    assignVelocityLimits(path, *analytic_curvatures, frenet_converter, config);
  } else {
    assignVelocityLimits(path, frenet_converter, config);
  }
  smoothVelocityProfile(path, start_velocity_mps, config);
}

} // namespace

PathProcessingResult processSelectedPath(
  const std::vector<Point> & crude_path,
  const std::vector<FrenetPoint> & anchors,
  const Odometry & odom,
  const FrenetConverter & frenet_converter,
  const CollisionChecker & collision_checker,
  const OccupancyGrid & grid,
  const LocalFrenetPlannerConfig & config)
{
  PathProcessingResult result;
  result.path = crude_path;
  const std::vector<double> * spline_curvatures = nullptr;
  std::vector<double> accepted_spline_curvatures;

  switch (config.refinement_mode) {
    case RefinementMode::NONE:
      // Intentionally publish crude geometry; not a refinement failure.
      result.refinement_succeeded = true;
      result.used_crude_fallback = false;
      break;

    case RefinementMode::ANGLE_SMOOTHING: {
        const FrenetRefinementResult refined =
          rebuildFrenetGeometry(anchors, frenet_converter, config);
        if (refined.success &&
          validateRefinedGeometry(
          refined.path, refined.headings, frenet_converter, collision_checker, grid, config))
        {
          result.path = refined.path;
          result.refinement_succeeded = true;
          result.used_crude_fallback = false;
        } else {
          result.path = crude_path;
          result.refinement_succeeded = false;
          result.used_crude_fallback = true;
        }
        break;
      }

    case RefinementMode::SPLINE: {
      // Frenet angle smoothing is intentionally never run in this mode.
        const SplineRefinementResult refined =
          refineSplineGeometry(anchors, odom, frenet_converter, config);
        if (refined.success &&
          validateRefinedGeometry(
          refined.path, refined.headings, frenet_converter, collision_checker, grid, config))
        {
          result.path = refined.path;
          accepted_spline_curvatures = refined.curvatures;
          spline_curvatures = &accepted_spline_curvatures;
          result.refinement_succeeded = true;
          result.used_crude_fallback = false;
        } else {
          result.path = crude_path;
          result.refinement_succeeded = false;
          result.used_crude_fallback = true;
        }
        break;
      }
  }

  applyVelocityPipeline(
    result.path, odom.velocity, frenet_converter, config, spline_curvatures);
  return result;
}

} // namespace local_planning
