#ifndef LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
#define LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP

#include "local_planning/core/types.hpp"
#include "local_planning/curves/curve_connection_generator.hpp"
#include "local_planning/reference/raceline_reference.hpp"

#include <vector>

namespace local_planning
{

using Path = std::vector<CurveSample>;

// Only values needed by later candidate selection live beside the geometry.
// Construction diagnostics intentionally remain inside the curve generator.
struct ManeuverCandidate
{
  Path path;
  double target_d = 0.0;
  double maneuver_distance_m = 0.0;
  double max_offset_deviation_m = 0.0;
};

struct ManeuverConfig
{
  double horizon_m = 6.0;

  double collision_circle_radius_m = 0.20;
  std::vector<double> overtake_s_offsets_from_opponent_rear_m{0.0, 0.5, 1.0};
  std::vector<double> passing_d_magnitudes_m{0.55, 0.75};
  std::vector<double> overtake_heading_offsets_rad{-0.15, 0.0, 0.15};
  std::vector<double> overtake_curvature_multipliers{0.0, 0.5, 1.0};
  std::vector<double> pass_transition_distances_m{6.0, 3.0, 1.0};
  std::vector<double> merge_completion_distances_m{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

  double sideDeadbandM() const {return 2.0 * collision_circle_radius_m;}
};

class ManeuverBuilder
{
public:
  ManeuverBuilder(
    const RacelineReference & reference,
    const CurveConnectionGenerator & curve_generator,
    ManeuverConfig config);

  std::vector<ManeuverCandidate> overtake(
    const BoundaryState & ego,
    double ego_s,
    double ego_d,
    double opponent_rear_s) const;
  std::vector<ManeuverCandidate> pass(
    const BoundaryState & ego, double ego_s, double ego_d) const;
  std::vector<ManeuverCandidate> recover(
    const BoundaryState & ego, double ego_s, double ego_d) const;
  std::vector<ManeuverCandidate> merge(const BoundaryState & ego, double ego_s) const;
  const ManeuverConfig & config() const {return config_;}

private:
  bool boundary(
    double s,
    double d,
    double heading_offset,
    BoundaryState & result,
    double curvature_multiplier = 1.0) const;
  bool connect(
    Path & path,
    const BoundaryState & start,
    const BoundaryState & end,
    BoundaryState * actual_end = nullptr) const;
  bool appendTail(Path & path, double start_s, double distance, double d) const;
  bool staysOnSide(const Path & path, double ego_s, int side, bool allow_start_center) const;
  double maximumOffsetDeviation(const Path & path, double ego_s, double target_d) const;
  std::vector<double> offsets(int side) const;
  double preferredOffset(double ego_d) const;
  int sideOf(double d) const;

  const RacelineReference & reference_;
  const CurveConnectionGenerator & curve_generator_;
  ManeuverConfig config_;
};

} // namespace local_planning

#endif // LOCAL_PLANNING_MANEUVERS_MANEUVER_BUILDER_HPP
