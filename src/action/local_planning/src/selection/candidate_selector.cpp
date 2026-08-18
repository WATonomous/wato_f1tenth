#include "local_planning/selection/candidate_selector.hpp"

#include "local_planning/core/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{
constexpr double kTolerance = kGridEps;
constexpr std::size_t kKeyCount = 4;

bool valid(const EvaluatedCandidate & candidate)
{
  return candidate.candidate_index >= 0 && candidate.velocity_feasible &&
         (candidate.collision.status == CollisionStatus::FREE ||
         candidate.collision.status == CollisionStatus::SOFT_INFLATION);
}

// How far into the soft inflation a candidate goes, in metres.  FREE means
// clearance above the inflation distance, so every free candidate ties at
// exactly 0 and the geometry keys decide among them; every soft candidate
// scores above 0 and therefore loses to any free one.  Inside the soft band the
// actual clearance orders the candidates, because "grazes the inflation" and
// "nearly touches" are not the same answer and the old binary rank could not
// tell them apart.
double inflationDepth(const EvaluatedCandidate & candidate, double soft_inflation_distance_m)
{
  if (candidate.collision.status == CollisionStatus::FREE) {
    return 0.0;
  }
  return std::max(
    kTolerance, soft_inflation_distance_m - candidate.collision.minimum_clearance_m);
}

// Lower is better in every slot.  One comparator covers all three intents:
// MERGE commands d = 0 everywhere, so its geometry keys are constant and it
// falls through to time on its own, with no intent test needed here.
using SelectionKey = std::array<double, kKeyCount>;

SelectionKey keyFor(
  const ManeuverCandidate & geometry,
  const EvaluatedCandidate & evaluated,
  double soft_inflation_distance_m)
{
  return {
    inflationDepth(evaluated, soft_inflation_distance_m),
    std::abs(geometry.passing_d),
    std::abs(geometry.terminal_d),
    evaluated.traversal_time_s};
}

// Strictly better on the first key that differs by more than kTolerance.
// Exact ties fall through to first-seen, which keeps enumeration order as the
// final tiebreak exactly as the hand-rolled loops did.
bool better(const SelectionKey & candidate, const SelectionKey & best)
{
  for (std::size_t i = 0; i < kKeyCount; ++i) {
    if (candidate[i] < best[i] - kTolerance) {
      return true;
    }
    if (candidate[i] > best[i] + kTolerance) {
      return false;
    }
  }
  return false;
}

}  // namespace

CandidateSelector::CandidateSelector(double soft_inflation_distance_m)
: soft_inflation_distance_m_(soft_inflation_distance_m)
{
}

int CandidateSelector::select(
  const std::vector<ManeuverCandidate> & pool,
  const std::vector<EvaluatedCandidate> & evaluated) const
{
  int best_index = -1;
  SelectionKey best_key{};
  for (const auto & candidate : evaluated) {
    if (!valid(candidate)) {continue;}
    const auto & geometry = pool.at(static_cast<std::size_t>(candidate.candidate_index));
    const SelectionKey key = keyFor(geometry, candidate, soft_inflation_distance_m_);
    if (best_index < 0 || better(key, best_key)) {
      best_index = candidate.candidate_index;
      best_key = key;
    }
  }
  return best_index;
}

}  // namespace local_planning
