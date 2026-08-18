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
  // Separate from max_decel_mps2 so braking may be harder than the nominal
  // profile is allowed to plan.
  double decel_mps2 = 5.0;
  // Floor, not zero: a stopped car cannot steer itself back onto the line.
  double min_velocity_mps = 1.0;
  double friction_coeff = 1.0;
  // The steering stop, in radians at the wheel.  With the wheelbase this is the
  // tightest arc the car can physically hold; friction supplies the other, and
  // the binding one wins.
  double max_steering_angle_rad = 0.52;
  double wheelbase_m = 0.33;
  // Carrot distance along the reference.  Short chases the line hard and
  // oscillates; long converges lazily.
  std::vector<double> pursuit_lookaheads_m{1.0, 2.0, 3.0};
  // Fraction of the physical turning budget each fan member may spend.  1.0
  // cuts toward the line as hard as the car can hold at that speed, 0.0 is a
  // straight brake.  The hard cut is the one we want and also the one most
  // likely to be collision-rejected, so a gentler member exists to survive it.
  std::vector<double> effort_levels{1.0, 0.0};

  // Tightest arc holdable at speed v: the steering stop, and the friction
  // circle.  v falls along a braking path, so this grows -- which is the whole
  // reason braking recovers the racing line.
  double allowedCurvature(double v) const;
};

struct BrakingArcParams
{
  double effort = 0.0;
  double lookahead_m = 0.0;
};

// The braking family.  Unlike every other family this is not a Frenet
// connection: a car that is off-line, yawed, and too fast has no feasible
// quintic back to d = 0, which is precisely the situation braking exists for.
// Instead it forward-integrates the bicycle model under the friction-limited
// curvature budget, chasing a carrot that slides along the racing line.  The
// racing line is the safest aim point by construction -- nothing static
// occupies it, and anything moving on it is clearing while we slow.
class BrakingPathGenerator
{
public:
  BrakingPathGenerator(const RacelineReference & reference, BrakingConfig config);

  // One candidate per effort/lookahead pair, in effort-major configuration
  // order. Empty only when the reference is unusable; notably it does not
  // require valid track widths.
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
