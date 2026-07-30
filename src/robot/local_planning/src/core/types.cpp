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

} // namespace local_planning
