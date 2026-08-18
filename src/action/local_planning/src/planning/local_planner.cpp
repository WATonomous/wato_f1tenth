#include "local_planning/planning/local_planner.hpp"

#include "local_planning/core/scoped_timer.hpp"
#include "local_planning/speed/velocity_profile.hpp"
#include "worker_pool.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <tuple>
#include <vector>

namespace local_planning
{
PublishedPathChoice choosePublishedPath(
  ExecutedMode planned_mode,
  bool selected_available,
  bool held_available,
  bool held_collision_usable,
  double hold_age_s,
  double path_min_hold_s,
  double path_max_hold_s)
{
  const bool held_usable = held_available && held_collision_usable && hold_age_s >= 0.0 &&
    hold_age_s < path_max_hold_s;
  if (planned_mode == ExecutedMode::HELD_PATH) {
    return held_usable ? PublishedPathChoice::HELD : PublishedPathChoice::NONE;
  }
  if (planned_mode == ExecutedMode::MANEUVER) {
    if (held_usable && hold_age_s < path_min_hold_s) {
      return PublishedPathChoice::HELD;
    }
    return selected_available ? PublishedPathChoice::SELECTED : PublishedPathChoice::NONE;
  }
  if (planned_mode == ExecutedMode::BRAKING_FALLBACK ||
    planned_mode == ExecutedMode::BRAKING_UNAVAILABLE)
  {
    return selected_available ? PublishedPathChoice::SELECTED : PublishedPathChoice::NONE;
  }
  return PublishedPathChoice::NONE;
}

namespace
{
void appendCandidates(
  LocalPlanResult & result,
  std::vector<ManeuverCandidate> candidates,
  CandidateSource source)
{
  const int first = static_cast<int>(result.pool.size());
  result.pool.insert(result.pool.end(),
    std::make_move_iterator(candidates.begin()),
    std::make_move_iterator(candidates.end()));
  for (int i = first; i < static_cast<int>(result.pool.size()); ++i) {
    EvaluatedCandidate evaluated;
    evaluated.candidate_index = i;
    evaluated.source = source;
    result.evaluated.push_back(evaluated);
  }
}

void fillSelectedMetrics(
  LocalPlanResult & result)
{
  if (result.selected_index < 0) {return;}
  const auto & candidate = result.pool.at(static_cast<std::size_t>(result.selected_index));
  const auto eval = std::find_if(result.evaluated.begin(), result.evaluated.end(),
      [&result](const auto & item) {return item.candidate_index == result.selected_index;});
  if (eval != result.evaluated.end()) {
    result.decision.clearance_class = eval->collision.status;
    result.decision.minimum_clearance_m = eval->collision.minimum_clearance_m;
    result.decision.candidate_source = eval->source;
  }
  if (candidate.path.empty()) {return;}
  result.decision.selected_offset_tail = candidate.uses_offset_tail;
  result.decision.selected_max_abs_d_m = candidate.max_abs_d_m;
  result.decision.min_speed_mps = std::numeric_limits<double>::infinity();
  for (const auto & sample : candidate.path) {
    result.decision.max_abs_curvature_inv_m = std::max(
      result.decision.max_abs_curvature_inv_m, std::abs(sample.curvature));
    result.decision.min_speed_mps = std::min(result.decision.min_speed_mps, sample.speed);
    result.decision.max_speed_mps = std::max(result.decision.max_speed_mps, sample.speed);
  }
  result.decision.terminal_d_m = candidate.terminal_d;
}
}  // namespace

LocalPlanner::LocalPlanner(
  const RacelineReference & reference,
  const ManeuverBuilder & builder,
  VehicleGeometry vehicle_geometry,
  GridPolicy grid_policy,
  CollisionConfig collision_config,
  VelocityProfileConfig velocity_config,
  BrakingConfig braking_config)
: reference_(reference), builder_(builder), vehicle_geometry_(vehicle_geometry),
  velocity_config_(velocity_config),
  collision_checker_(vehicle_geometry_, grid_policy, collision_config),
  track_bounds_checker_(reference_, vehicle_geometry_),
  braking_generator_(reference_, braking_config),
  selector_(collision_config.soft_inflation_distance_m)
{
}

void LocalPlanner::buildGridCache(OccupancyGrid & grid) const
{
  collision_checker_.buildEuclideanTransform(grid);
}

LocalPlanResult LocalPlanner::plan(
  const TacticalState & state,
  const BoundaryState & ego,
  const OccupancyGrid & grid,
  bool held_path_usable) const
{
  LocalPlanResult result;
  const ScopedTimer cycle_timer{result.decision.cycle_time_ms};
  result.decision.requested_intent = state.intent;
  result.decision.proposed_intent = state.proposed_intent;
  result.decision.executed_intent = state.intent;
  result.decision.transition_class = state.transition_class;
  result.decision.transition_reason = state.transition_reason;
  result.decision.pending_transition_s = state.pending_duration_s;
  result.decision.pending_grid_count = state.pending_grid_count;
  result.decision.merge_probe_valid_cycles = state.merge_probe_valid_cycles;
  result.decision.merge_probe_available = false;
  result.decision.costmap_sequence = state.costmap_sequence;
  result.decision.costmap_stamp_s = state.costmap_stamp_s;
  result.decision.opponent_observation_sequence = state.opponent_observation_sequence;
  if (state.intent == PlannerIntent::PASS &&
    state.committed_transition_reason == TransitionReason::OPPONENT_CLEARANCE_LOST)
  {
    result.decision.recovery_reason = RecoveryReason::OPPONENT_CLEARANCE_LOST;
  }
  result.decision.relative_position = state.relative_position;
  result.decision.opponent_detected = state.opponent.detected;
  result.decision.opponent_gap_m = state.opponent.gap_m;
  result.decision.ego_s_m = state.ego_s;
  result.decision.ego_d_m = state.ego_d;
  result.decision.heading_error_rad = state.heading_error_rad;
  result.decision.raceline_compatible = state.raceline_compatible;
  result.decision.projection_seed_was_stale = state.ego_seed_was_stale;
  result.decision.projection_heading_check_relaxed = state.ego_heading_check_relaxed;
  result.decision.start_curvature_inv_m = ego.curvature;
  result.decision.start_curvature_from_steering = std::abs(ego.curvature) > 0.0;
  result.decision.track_bounds_ready = reference_.trackWidthsValid();
  const SustainableBounds bounds = reference_.rawBounds(state.ego_s);
  result.decision.sustainable_left_m = bounds.left_magnitude;
  result.decision.sustainable_right_m = bounds.right_magnitude;

  if (state.intent == PlannerIntent::FOLLOW_RACING_LINE) {
    return result;
  }

  auto appendGenerated = [&](CandidateSource source, auto generator) {
      appendCandidates(result, generator(), source);
    };

  if (!result.decision.track_bounds_ready) {
    // FOLLOW remains available through the global raceline consumer. Local
    // maneuvers require the matching, index-aligned width message.
  } else if (state.intent == PlannerIntent::OVERTAKE) {
    appendGenerated(CandidateSource::OVERTAKE, [&]() {
        return builder_.overtake(
          ego, state.ego_s, state.ego_d, state.opponent.s);
    });
  } else if (state.intent == PlannerIntent::PASS) {
    appendGenerated(CandidateSource::PASS_PREFERRED, [&]() {
        return builder_.pass(ego, state.ego_s, state.ego_d);
    });
  } else {
    appendGenerated(CandidateSource::MERGE, [&]() {
        return builder_.merge(ego, state.ego_s, state.ego_d);
    });
  }

  auto evaluateFrom = [&](std::size_t first, PlannerIntent profile_intent) {
      const std::size_t count = result.evaluated.size() - first;
      parallelFor(count, [&](std::size_t k) {
          auto & evaluated = result.evaluated[first + k];
          auto & candidate = result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
          evaluated.collision = collision_checker_.collisionCheck(candidate.path, grid);
        });

      std::vector<TrackBoundsCheckResult> width_checks(count);
      parallelFor(count, [&](std::size_t k) {
          auto & evaluated = result.evaluated[first + k];
          if (evaluated.collision.status == CollisionStatus::COLLISION ||
          evaluated.collision.status == CollisionStatus::OUT_OF_GRID)
          {
            return;
          }
          auto & candidate = result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
          width_checks[k] = track_bounds_checker_.check(candidate.path, grid);
          evaluated.track_bounds_ok = width_checks[k].ok;
        });

      const double terminal_s = reference_.wrapS(state.ego_s + builder_.config().horizon_m);
      for (std::size_t k = 0; k < count; ++k) {
        auto & evaluated = result.evaluated[first + k];
        auto & candidate = result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
        if (evaluated.collision.status == CollisionStatus::COLLISION) {
          ++result.decision.collision_rejected;
          continue;
        }
        if (evaluated.collision.status == CollisionStatus::OUT_OF_GRID) {
          ++result.decision.out_of_grid_rejected;
          continue;
        }
        if (!evaluated.track_bounds_ok) {
          ++result.decision.track_bounds_rejected;
          continue;
        }
        const auto velocity = assignVelocityProfile(candidate.path, ego.speed, state.ego_s,
            terminal_s, profile_intent, reference_, velocity_config_);
        evaluated.velocity_feasible = velocity.feasible;
        evaluated.traversal_time_s = velocity.traversal_time_s;
        if (!velocity.feasible) {
          ++result.decision.velocity_rejected;
        } else {
          ++result.decision.valid_candidate_count;
        }
      }
    };

  evaluateFrom(0, state.intent);
  // Lazy generation, stated directly.  This used to run the PASS *ranking* just
  // to answer it, which meant selectPass() was called twice a cycle for two
  // unrelated purposes and the tier machinery had to survive to serve this one.
  // The question was only ever whether the recovery family is worth generating.
  if (state.intent == PlannerIntent::PASS) {
    const bool preferred_is_free = std::any_of(
      result.evaluated.begin(), result.evaluated.end(),
      [](const EvaluatedCandidate & item) {
        return item.source == CandidateSource::PASS_PREFERRED &&
               item.velocity_feasible &&
               item.collision.status == CollisionStatus::FREE;
      });
    if (!preferred_is_free) {
      const std::size_t first = result.evaluated.size();
      appendGenerated(CandidateSource::PASS_RECOVERY, [&]() {
          return builder_.recover(ego, state.ego_s, state.ego_d);
      });
      evaluateFrom(first, PlannerIntent::PASS);
    }
  }
  if (state.intent == PlannerIntent::PASS &&
    state.proposed_intent == PlannerIntent::MERGE)
  {
    const std::size_t first = result.evaluated.size();
    appendGenerated(CandidateSource::MERGE_PROBE, [&]() {
        return builder_.merge(ego, state.ego_s, state.ego_d);
    });
    evaluateFrom(first, PlannerIntent::MERGE);
    std::vector<EvaluatedCandidate> probes(
      result.evaluated.begin() + static_cast<std::ptrdiff_t>(first), result.evaluated.end());
    result.merge_probe_index = selector_.select(result.pool, probes);
    result.decision.merge_probe_available = result.merge_probe_index >= 0;
  }
  std::vector<EvaluatedCandidate> executable;
  executable.reserve(result.evaluated.size());
  for (const auto & evaluated : result.evaluated) {
    if (evaluated.source != CandidateSource::MERGE_PROBE) {
      executable.push_back(evaluated);
    }
  }
  result.selected_index = selector_.select(result.pool, executable);

  if (state.intent == PlannerIntent::MERGE && result.selected_index < 0 &&
    result.decision.track_bounds_ready)
  {
    const std::size_t recovery_first = result.evaluated.size();
    appendGenerated(CandidateSource::PASS_PREFERRED, [&]() {
        return builder_.pass(ego, state.ego_s, state.ego_d);
    });
    evaluateFrom(recovery_first, PlannerIntent::PASS);
    const bool preferred_is_free = std::any_of(
      result.evaluated.begin() + static_cast<std::ptrdiff_t>(recovery_first),
      result.evaluated.end(), [](const EvaluatedCandidate & item) {
        return item.source == CandidateSource::PASS_PREFERRED && item.velocity_feasible &&
               item.collision.status == CollisionStatus::FREE;
      });
    if (!preferred_is_free) {
      const std::size_t fallback_first = result.evaluated.size();
      appendGenerated(CandidateSource::PASS_RECOVERY, [&]() {
          return builder_.recover(ego, state.ego_s, state.ego_d);
      });
      evaluateFrom(fallback_first, PlannerIntent::PASS);
    }
    std::vector<EvaluatedCandidate> recovery;
    for (std::size_t i = recovery_first; i < result.evaluated.size(); ++i) {
      recovery.push_back(result.evaluated[i]);
    }
    result.selected_index = selector_.select(result.pool, recovery);
    if (result.selected_index >= 0) {
      result.decision.executed_intent = PlannerIntent::PASS;
      result.decision.recovery_reason = RecoveryReason::MERGE_PATH_UNAVAILABLE;
    }
  }

  std::vector<double> costs;
  for (const auto & evaluated : result.evaluated) {
    if (evaluated.velocity_feasible) {costs.push_back(evaluated.traversal_time_s);}
  }
  if (!costs.empty()) {
    std::sort(costs.begin(), costs.end());
    result.decision.best_cost_s = costs.front();
    result.decision.median_cost_s = costs[costs.size() / 2];
  }
  if (result.selected_index >= 0) {
    result.decision.executed_mode = ExecutedMode::MANEUVER;
  } else if (held_path_usable) {
    result.decision.executed_mode = ExecutedMode::HELD_PATH;
  } else {
    selectBraking(result, ego, state.ego_s, state.ego_d, grid);
  }

  fillSelectedMetrics(result);
  if (result.decision.executed_mode == ExecutedMode::BRAKING_FALLBACK ||
    result.decision.executed_mode == ExecutedMode::BRAKING_UNAVAILABLE)
  {
    result.decision.candidate_source = CandidateSource::BRAKING;
  }
  result.decision.generated_count = static_cast<uint32_t>(result.pool.size());
  return result;
}

void LocalPlanner::selectBraking(
  LocalPlanResult & result,
  const BoundaryState & ego,
  double ego_s,
  double ego_d,
  const OccupancyGrid & grid) const
{
  const std::size_t first = result.evaluated.size();
  appendCandidates(
    result, braking_generator_.generate(ego, ego_s, ego_d), CandidateSource::BRAKING);
  const std::size_t count = result.evaluated.size() - first;
  if (count == 0) {
    result.decision.executed_mode = ExecutedMode::BRAKING_UNAVAILABLE;
    result.decision.recovery_reason = RecoveryReason::NO_SAFE_LOCAL_PATH;
    return;
  }

  parallelFor(count, [&](std::size_t k) {
      auto & evaluated = result.evaluated[first + k];
      const auto & candidate = result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
      evaluated.collision = collision_checker_.collisionCheck(candidate.path, grid);
      evaluated.track_bounds_ok = track_bounds_checker_.check(candidate.path, grid).ok;
      // Deliberately no assignVelocityProfile: braking owns its speeds, and the
      // nominal profiler rejects an infeasible start speed -- which is the one
      // condition braking exists to answer.
    });

  // Only a seen obstacle disqualifies an arc.  Running off the grid does not:
  // the horizon is 4 m and the costmap is a 15 m window, so leaving it is
  // routine, and refusing to brake because the map ran out is worse than
  // braking into a cell nobody has looked at.  Ranking still prefers the arcs
  // the costmap could vouch for -- OUT_OF_GRID carries -inf clearance, so it
  // sorts below anything seen and free.
  //
  // The honest version of this is a Frenet bounds lookup: convert the arc to
  // (s, d) and check it against the raceline width table where the grid has
  // nothing to say. Worth doing if unseen tails start mattering; for a
  // last-resort mode it is more machinery than the decision deserves.
  const auto score = [&](std::size_t k) {
      const auto & evaluated = result.evaluated[first + k];
      const auto & candidate = result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
      const bool free = evaluated.collision.status == CollisionStatus::FREE;
      const bool usable = evaluated.collision.status != CollisionStatus::COLLISION;
      return std::make_tuple(
        usable, free, evaluated.track_bounds_ok,
        evaluated.collision.minimum_clearance_m, -std::abs(candidate.terminal_d));
    };
  std::size_t best = 0;
  for (std::size_t k = 1; k < count; ++k) {
    if (score(k) > score(best)) {best = k;}
  }

  const auto & winner = result.evaluated[first + best];
  result.selected_index = winner.candidate_index;
  const auto params = braking_generator_.arcParams();
  if (best < params.size()) {
    result.decision.braking_effort = params[best].effort;
    result.decision.braking_lookahead_m = params[best].lookahead_m;
  }
  if (std::get<0>(score(best))) {
    result.decision.executed_mode = ExecutedMode::BRAKING_FALLBACK;
    result.decision.recovery_reason = RecoveryReason::BRAKING_FALLBACK;
    return;
  }
  // Nowhere free to go.  Publish the geometry anyway with the speeds zeroed:
  // stopping with a coherent steering angle beats the controller's dead_stop(),
  // which zeroes the wheel too and abandons the corner mid-turn.
  for (auto & sample : result.pool.at(static_cast<std::size_t>(result.selected_index)).path) {
    sample.speed = 0.0;
  }
  result.decision.executed_mode = ExecutedMode::BRAKING_UNAVAILABLE;
  result.decision.recovery_reason = RecoveryReason::NO_SAFE_LOCAL_PATH;
}

}  // namespace local_planning
