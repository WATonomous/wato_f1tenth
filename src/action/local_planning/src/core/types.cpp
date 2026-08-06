#include "local_planning/core/types.hpp"

namespace local_planning
{

std::string intentToString(PlannerIntent intent)
{
  switch (intent) {
    case PlannerIntent::FOLLOW_RACING_LINE:
      return "FOLLOW_RACING_LINE";
    case PlannerIntent::OVERTAKE:
      return "OVERTAKE";
    case PlannerIntent::PASS:
      return "PASS";
    case PlannerIntent::MERGE:
      return "MERGE";
  }
  return "UNKNOWN";
}

std::string relativePositionToString(RelativePosition position)
{
  switch (position) {
    case RelativePosition::NONE:
      return "NONE";
    case RelativePosition::BEHIND:
      return "BEHIND";
    case RelativePosition::OVERLAPPING:
      return "OVERLAPPING";
    case RelativePosition::AHEAD_NOT_CLEAR:
      return "AHEAD_NOT_CLEAR";
    case RelativePosition::AHEAD_AND_CLEAR:
      return "AHEAD_AND_CLEAR";
  }
  return "UNKNOWN";
}

} // namespace local_planning
