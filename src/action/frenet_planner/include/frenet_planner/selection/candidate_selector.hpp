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

#ifndef FRENET_PLANNER_SELECTION_CANDIDATE_SELECTOR_HPP
#define FRENET_PLANNER_SELECTION_CANDIDATE_SELECTOR_HPP

#include <vector>

#include "frenet_planner/collision/collision_checker.hpp"
#include "frenet_planner/maneuvers/maneuver_builder.hpp"

namespace frenet_planner
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

  int select(const std::vector<ManeuverCandidate> & pool, const std::vector<EvaluatedCandidate> & evaluated) const;

private:
  double soft_inflation_distance_m_ = 0.0;
};

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_SELECTION_CANDIDATE_SELECTOR_HPP
