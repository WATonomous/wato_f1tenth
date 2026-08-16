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
  BRAKING = 6
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
// tuple.
//
// Safety is binary everywhere: FREE beats SOFT_INFLATION and nothing more.
// minimum_clearance_m deliberately never enters ranking -- it stays in the
// no-candidate braking fallback, the one place a continuous margin is the right
// question.  Ranking on a float margin here would make traversal_time_s a
// tiebreak that never fires.
//
// OVERTAKE then ranks on geometry: (|passing_d|, |terminal_d|), because the
// offset that has to fit beside the opponent is what costs lap time, and the
// horizon offset breaks its ties.  Both legs of every OVERTAKE candidate are
// already on one side of the raceline, so ranking on |d| cannot pull a
// candidate across it.
//
// PASS and MERGE rank on time alone after safety.  "On our side" is enforced
// when PASS candidates are generated, not here.
class CandidateSelector
{
public:
  // Returns the index into pool, or -1 when nothing is both feasible and no
  // worse than SOFT_INFLATION.  intent selects the key tuple; anything that is
  // not OVERTAKE uses (safety, time).
  int select(
    PlannerIntent intent,
    const std::vector<ManeuverCandidate> & pool,
    const std::vector<EvaluatedCandidate> & evaluated) const;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_SELECTION_CANDIDATE_SELECTOR_HPP
