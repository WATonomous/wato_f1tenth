#ifndef LOCAL_PLANNING_SELECTION_CANDIDATE_SELECTOR_HPP
#define LOCAL_PLANNING_SELECTION_CANDIDATE_SELECTOR_HPP

#include "local_planning/collision/collision_checker.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"

#include <vector>

namespace local_planning
{

enum class CandidateSource : uint8_t
{
  NONE = 0,
  OVERTAKE = 1,
  PASS_PREFERRED = 2,
  PASS_RECOVERY = 3,
  MERGE_ALIGNMENT = 4,
  MERGE = 5,
  BRAKING = 6,
  MERGE_PROBE = 7
};

struct EvaluatedCandidate
{
  int candidate_index = -1;
  CandidateSource source = CandidateSource::NONE;
  CollisionCheckResult collision;
  bool velocity_feasible = false;
  double traversal_time_s = 0.0;
  bool track_bounds_ok = true;
};

// Picks one candidate per cycle by lexicographic comparison on a small key
// tuple: (inflation depth, |passing_d|, |terminal_d|, traversal time).
//
// Safety leads, and it is not a binary: a free candidate scores 0, and a soft
// one scores by how far into the inflation it actually reaches, so the shallow
// graze is preferred over the near-touch.  Free still beats soft outright,
// because free means clearance beyond the inflation distance and therefore a
// depth of exactly 0, which no soft candidate can reach.
//
// Geometry then ranks on (|passing_d|, |terminal_d|): the offset that has to
// fit beside the opponent is what costs lap time, and the horizon offset breaks
// its ties.  Every candidate's legs are already on one side of the raceline, so
// ranking on |d| cannot pull a candidate across it.
//
// One key serves every intent.  PASS offers each offset on its side and this
// picks the tightest clear one, which is also what stops a pass from ratcheting
// outward.  MERGE commands d = 0 on every candidate, so its geometry keys are
// constant and it falls through to traversal time by itself.  "On our side" is
// enforced when candidates are generated, not here.
class CandidateSelector
{
public:
  explicit CandidateSelector(double soft_inflation_distance_m);

  // Returns the index into pool, or -1 when nothing is both feasible and no
  // worse than SOFT_INFLATION.
  int select(
    const std::vector<ManeuverCandidate> & pool,
    const std::vector<EvaluatedCandidate> & evaluated) const;

private:
  double soft_inflation_distance_m_ = 0.0;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_SELECTION_CANDIDATE_SELECTOR_HPP
