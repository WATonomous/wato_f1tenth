#include "local_planning/selection/candidate_selector.hpp"

#include <array>
#include <cmath>
#include <cstddef>

namespace local_planning
{
namespace
{
constexpr double kTolerance = 1e-6;
constexpr std::size_t kKeyCount = 4;

bool valid(const EvaluatedCandidate & candidate)
{
  return candidate.candidate_index >= 0 && candidate.velocity_feasible &&
         (candidate.collision.status == CollisionStatus::FREE ||
         candidate.collision.status == CollisionStatus::SOFT_INFLATION);
}

int clearanceRank(const EvaluatedCandidate & candidate)
{
  return candidate.collision.status == CollisionStatus::FREE ? 0 : 1;
}

// Lower is better in every slot.  Unused slots are zero, which is why one
// comparator covers all three intents: PASS and MERGE simply tie on the two
// geometry keys and fall through to time.
using SelectionKey = std::array<double, kKeyCount>;

SelectionKey keyFor(
  PlannerIntent intent,
  const ManeuverCandidate & geometry,
  const EvaluatedCandidate & evaluated)
{
  const bool rank_on_geometry = intent == PlannerIntent::OVERTAKE;
  return {
    static_cast<double>(clearanceRank(evaluated)),
    rank_on_geometry ? std::abs(geometry.passing_d) : 0.0,
    rank_on_geometry ? std::abs(geometry.terminal_d) : 0.0,
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

int CandidateSelector::select(
  PlannerIntent intent,
  const std::vector<ManeuverCandidate> & pool,
  const std::vector<EvaluatedCandidate> & evaluated) const
{
  int best_index = -1;
  SelectionKey best_key{};
  for (const auto & candidate : evaluated) {
    if (!valid(candidate)) {continue;}
    const auto & geometry = pool.at(static_cast<std::size_t>(candidate.candidate_index));
    const SelectionKey key = keyFor(intent, geometry, candidate);
    if (best_index < 0 || better(key, best_key)) {
      best_index = candidate.candidate_index;
      best_key = key;
    }
  }
  return best_index;
}

}  // namespace local_planning
