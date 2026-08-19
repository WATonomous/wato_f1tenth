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

// Lexicographic rank: (inflation depth, |passing_d|, |terminal_d|, traversal time).
// FREE beats SOFT_INFLATION; shallower soft graze beats deeper.
class CandidateSelector
{
public:
  explicit CandidateSelector(double soft_inflation_distance_m);

  int select(
    const std::vector<ManeuverCandidate> & pool,
    const std::vector<EvaluatedCandidate> & evaluated) const;

private:
  double soft_inflation_distance_m_ = 0.0;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_SELECTION_CANDIDATE_SELECTOR_HPP
