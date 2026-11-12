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

#ifndef FRENET_PLANNER_SPEED_VELOCITY_PROFILE_HPP
#define FRENET_PLANNER_SPEED_VELOCITY_PROFILE_HPP

#include <vector>

#include "frenet_planner/core/types.hpp"
#include "frenet_planner/reference/raceline_reference.hpp"

namespace frenet_planner
{

struct VelocityProfileConfig
{
  double friction_coeff = 1.0;
  double min_velocity_mps = 0.0;
  double max_velocity_mps = 10.0;
  double max_accel_mps2 = 5.0;
  double max_decel_mps2 = 5.0;
  double overtake_speed_scale = 1.1;  // OVERTAKE/PASS interior; MERGE terminal unscaled
};

struct VelocityProfileResult
{
  bool feasible = false;
  double traversal_time_s = 0.0;
};

// Feasible speed profile on a complete path; writes speeds only when feasible.
VelocityProfileResult assignVelocityProfile(
  std::vector<CurveSample> & path,
  double start_velocity_mps,
  double start_raceline_s,
  double terminal_raceline_s,
  PlannerIntent intent,
  const RacelineReference & reference,
  const VelocityProfileConfig & config);

}  // namespace frenet_planner

#endif  // FRENET_PLANNER_SPEED_VELOCITY_PROFILE_HPP
