#ifndef LOCAL_PLANNING_PLANNING_BRAKING_PATH_GENERATOR_HPP
#define LOCAL_PLANNING_PLANNING_BRAKING_PATH_GENERATOR_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <vector>

namespace local_planning
{

struct BrakingConfig
{
  double horizon_m = 4.0;
  double sample_spacing_m = 0.1;
  double decel_mps2 = 5.0;           // may exceed nominal profile decel
  double min_velocity_mps = 1.0;     // floor; stopped car cannot steer back
  double friction_coeff = 1.0;
  double max_steering_angle_rad = 0.52;
  double wheelbase_m = 0.33;
  std::vector<double> pursuit_lookaheads_m{1.0, 2.0, 3.0};  // reference carrot distance
  std::vector<double> effort_levels{1.0, 0.0};              // turn budget fraction; 0 = straight brake

  double allowedCurvature(double v) const;  // min(steering, friction); grows as v falls
};

struct BrakingArcParams
{
  double effort = 0.0;
  double lookahead_m = 0.0;
};

// Bicycle-model fallback when no Frenet maneuver is feasible; chases raceline carrot while braking.
class BrakingPathGenerator
{
public:
  BrakingPathGenerator(const RacelineReference & reference, BrakingConfig config);

  std::vector<ManeuverCandidate> generate(
    const BoundaryState & ego, double ego_s, double ego_d) const;

  std::vector<BrakingArcParams> arcParams() const;

  const BrakingConfig & config() const {return config_;}

private:
  ManeuverCandidate integrate(
    const BoundaryState & ego, double ego_s, double ego_d,
    const BrakingArcParams & params) const;

  const RacelineReference & reference_;
  BrakingConfig config_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_PLANNING_BRAKING_PATH_GENERATOR_HPP
