// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "frenet_planner/core/types.hpp"

#include <string>

namespace frenet_planner
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

}  // namespace frenet_planner
