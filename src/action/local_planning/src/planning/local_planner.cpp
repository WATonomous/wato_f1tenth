#include "local_planning/planning/local_planner.hpp"

#include "local_planning/speed/velocity_profile.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>

namespace local_planning
{
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
  result.decision.min_speed_mps = std::numeric_limits<double>::infinity();
  for (const auto & sample : candidate.path) {
    result.decision.max_abs_curvature_inv_m = std::max(
      result.decision.max_abs_curvature_inv_m, std::abs(sample.curvature));
    result.decision.min_speed_mps = std::min(result.decision.min_speed_mps, sample.speed);
    result.decision.max_speed_mps = std::max(result.decision.max_speed_mps, sample.speed);
  }
  result.decision.terminal_d_m = candidate.target_d;
}
}  // namespace

LocalPlanner::LocalPlanner(
  const RacelineReference & reference,
  const ManeuverBuilder & builder,
  VehicleGeometry vehicle_geometry,
  GridPolicy grid_policy,
  CollisionConfig collision_config,
  VelocityProfileConfig velocity_config)
: reference_(reference), builder_(builder), vehicle_geometry_(vehicle_geometry),
  velocity_config_(velocity_config),
  collision_checker_(vehicle_geometry_, grid_policy, collision_config),
  track_bounds_checker_(reference_, vehicle_geometry_)
{
}

void LocalPlanner::buildGridCache(OccupancyGrid & grid) const
{
  collision_checker_.buildEuclideanTransform(grid);
}

LocalPlanResult LocalPlanner::plan(
  const TacticalState & state,
  const BoundaryState & ego,
  const OccupancyGrid & grid) const
{
  const auto started = std::chrono::steady_clock::now();
  const auto elapsedMs = [](const auto begin) {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    };
  LocalPlanResult result;
  result.decision.requested_intent = state.intent;
  result.decision.relative_position = state.relative_position;
  result.decision.opponent_detected = state.opponent.detected;
  result.decision.opponent_gap_m = state.opponent.gap_m;
  result.decision.ego_s_m = state.ego_s;
  result.decision.ego_d_m = state.ego_d;
  result.decision.heading_error_rad = state.heading_error_rad;
  result.decision.raceline_compatible = state.raceline_compatible;
  result.decision.projection_seed_was_stale = state.ego_seed_was_stale;
  result.decision.start_curvature_inv_m = ego.curvature;
  result.decision.start_curvature_from_steering = std::abs(ego.curvature) > 0.0;
  result.decision.track_bounds_ready = reference_.trackWidthsValid();
  const SustainableBounds bounds = reference_.rawBounds(state.ego_s);
  result.decision.sustainable_left_m = bounds.left_magnitude;
  result.decision.sustainable_right_m = bounds.right_magnitude;

  if (state.intent == PlannerIntent::FOLLOW_RACING_LINE) {
    result.decision.cycle_time_ms = elapsedMs(started);
    return result;
  }

  builder_.resetStationHintStats();
  uint64_t unseen_hint_samples = 0;
  uint64_t unseen_hint_fallbacks = 0;
  auto appendGenerated = [&](CandidateSource source, auto generator) {
      const auto generation_started = std::chrono::steady_clock::now();
      auto candidates = generator();
      result.profile.candidate_generation_ms += elapsedMs(generation_started);
      appendCandidates(result, std::move(candidates), source);
    };

  PlannerIntent profile_intent = state.intent;
  if (!result.decision.track_bounds_ready) {
    // FOLLOW remains available through the global raceline consumer. Local
    // maneuvers require the matching, index-aligned width message.
  } else if (state.intent == PlannerIntent::OVERTAKE) {
    appendGenerated(CandidateSource::OVERTAKE, [&]() {
        return builder_.overtake(
          ego, state.ego_s, state.ego_d, state.opponent.s);
    });
  } else if (state.intent == PlannerIntent::PASS &&
    std::abs(state.ego_d) <= vehicle_geometry_.fullWidthM())
  {
    appendGenerated(CandidateSource::MERGE_ALIGNMENT, [&]() {
        return builder_.merge(ego, state.ego_s);
    });
    profile_intent = PlannerIntent::MERGE;
  } else if (state.intent == PlannerIntent::PASS) {
    appendGenerated(CandidateSource::PASS_PREFERRED, [&]() {
        return builder_.pass(ego, state.ego_s, state.ego_d);
    });
  } else {
    appendGenerated(CandidateSource::MERGE, [&]() {
        return builder_.merge(ego, state.ego_s);
    });
  }

  auto evaluateFrom = [&](std::size_t first) {
      for (std::size_t i = first; i < result.evaluated.size(); ++i) {
        auto & evaluated = result.evaluated[i];
        auto & candidate = result.pool.at(static_cast<std::size_t>(evaluated.candidate_index));
        const auto collision_started = std::chrono::steady_clock::now();
        evaluated.collision = collision_checker_.collisionCheck(candidate.path, grid);
        result.profile.collision_check_ms += elapsedMs(collision_started);
        result.profile.collision_poses_checked += evaluated.collision.checked_poses;
        if (evaluated.collision.status == CollisionStatus::COLLISION) {
          ++result.decision.collision_rejected;
          continue;
        }
        if (evaluated.collision.status == CollisionStatus::OUT_OF_GRID) {
          ++result.decision.out_of_grid_rejected;
          continue;
        }
        const TrackBoundsCheckResult width_check = track_bounds_checker_.check(
          candidate.path, grid);
        unseen_hint_samples += width_check.station_hint_samples;
        unseen_hint_fallbacks += width_check.station_hint_fallbacks;
        if (!width_check.ok) {
          evaluated.track_bounds_ok = false;
          ++result.decision.track_bounds_rejected;
          continue;
        }
        const double terminal_s = reference_.wrapS(
          state.ego_s + builder_.config().horizon_m);
        const auto velocity_started = std::chrono::steady_clock::now();
        const auto velocity = assignVelocityProfile(candidate.path, ego.speed, state.ego_s,
            terminal_s, profile_intent, reference_, velocity_config_);
        result.profile.velocity_profile_ms += elapsedMs(velocity_started);
        evaluated.velocity_feasible = velocity.feasible;
        evaluated.traversal_time_s = velocity.traversal_time_s;
        if (!velocity.feasible) {
          ++result.decision.velocity_rejected;
        } else {
          ++result.decision.valid_candidate_count;
        }
      }
    };

  evaluateFrom(0);
  if (state.intent == PlannerIntent::PASS && profile_intent == PlannerIntent::PASS) {
    const int preferred = selector_.selectPass(result.pool, result.evaluated);
    const auto eval = std::find_if(result.evaluated.begin(), result.evaluated.end(),
        [preferred](const auto & item) {return item.candidate_index == preferred;});
    if (preferred < 0 || eval == result.evaluated.end() ||
      eval->collision.status != CollisionStatus::FREE)
    {
      const std::size_t first = result.evaluated.size();
      appendGenerated(CandidateSource::PASS_RECOVERY, [&]() {
          return builder_.recover(ego, state.ego_s, state.ego_d);
      });
      evaluateFrom(first);
    }
  }
  const auto station_hints = builder_.stationHintStats();
  result.profile.station_hint_samples = station_hints.samples + unseen_hint_samples;
  result.profile.station_hint_fallbacks = station_hints.fallbacks + unseen_hint_fallbacks;
  result.decision.generated_count = static_cast<uint32_t>(result.pool.size());
  for (const auto & candidate : result.pool) {
    const auto sample_count = static_cast<uint32_t>(candidate.path.size());
    result.profile.total_path_samples += sample_count;
    result.profile.max_path_samples = std::max(result.profile.max_path_samples, sample_count);
  }

  const auto selection_started = std::chrono::steady_clock::now();
  if (state.intent == PlannerIntent::OVERTAKE) {
    result.selected_index = selector_.selectOvertake(result.pool, result.evaluated);
  } else if (state.intent == PlannerIntent::PASS && profile_intent == PlannerIntent::PASS) {
    result.selected_index = selector_.selectPass(result.pool, result.evaluated);
  } else {
    result.selected_index = selector_.selectMerge(result.pool, result.evaluated);
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
  result.profile.selection_ms += elapsedMs(selection_started);

  const auto finalization_started = std::chrono::steady_clock::now();
  if (result.selected_index >= 0) {
    result.decision.executed_mode = ExecutedMode::MANEUVER;
  } else {
    const EvaluatedCandidate * safest = nullptr;
    for (const auto & evaluated : result.evaluated) {
      if (!evaluated.track_bounds_ok ||
        (evaluated.collision.status != CollisionStatus::FREE &&
        evaluated.collision.status != CollisionStatus::SOFT_INFLATION))
      {
        continue;
      }
      if (!safest || evaluated.collision.minimum_clearance_m >
        safest->collision.minimum_clearance_m)
      {
        safest = &evaluated;
      }
    }
    if (safest) {
      result.selected_index = safest->candidate_index;
      auto & path = result.pool.at(static_cast<std::size_t>(result.selected_index)).path;
      for (auto & sample : path) {
        sample.speed = std::max(velocity_config_.min_velocity_mps,
            std::sqrt(std::max(0.0, ego.speed * ego.speed -
            2.0 * velocity_config_.max_decel_mps2 * sample.s)));
      }
      result.decision.executed_mode = ExecutedMode::BRAKING_FALLBACK;
      result.decision.candidate_source = CandidateSource::BRAKING;
    } else {
      result.decision.executed_mode = ExecutedMode::BRAKING_UNAVAILABLE;
    }
  }

  fillSelectedMetrics(result);
  if (result.decision.executed_mode == ExecutedMode::BRAKING_FALLBACK) {
    result.decision.candidate_source = CandidateSource::BRAKING;
  }
  result.profile.finalization_ms = elapsedMs(finalization_started);
  result.decision.cycle_time_ms = elapsedMs(started);
  return result;
}

}  // namespace local_planning
