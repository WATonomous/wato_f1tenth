#include "local_planning/ros/planner_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace local_planning
{
namespace
{
constexpr int kRareErrorThrottleMs = 10000;
constexpr double kStationHintFallbackWarnPct = 2.0;
constexpr std::size_t kMaxGridProfileSamples = 512;
}  // namespace

PlannerDiagnostics::PlannerDiagnostics(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  PlannerDiagnosticsConfig config)
: logger_(std::move(logger)), clock_(std::move(clock)), config_(std::move(config))
{
}

void PlannerDiagnostics::recordGridUpdate(double update_ms, const OccupancyGrid & grid)
{
  grid_width_ = grid.width;
  grid_height_ = grid.height;
  grid_resolution_ = grid.resolution;
  if (!config_.profiling_enabled) {
    return;
  }

  // Keep the most recent samples if an intent filter leaves the window
  // undrained for a long time. The update count remains uncapped.
  if (grid_profiling_window_.size() >= kMaxGridProfileSamples) {
    grid_profiling_window_.erase(grid_profiling_window_.begin());
  }
  ++grid_updates_since_report_;
  grid_profiling_window_.push_back(update_ms);
}

void PlannerDiagnostics::recordCycle(
  PlannerCycleProfile sample,
  const PlannerDecisionData * decision,
  bool path_published,
  bool steering_fresh)
{
  if (decision != nullptr) {
    noteTransitions(*decision, path_published, steering_fresh, sample);
  }
  recordProfile(std::move(sample));
}

void PlannerDiagnostics::recordProfile(PlannerCycleProfile sample)
{
  if (!config_.profiling_enabled) {
    return;
  }
  if (config_.profiling_intent_filter && *config_.profiling_intent_filter != sample.intent) {
    return;
  }

  // Bucket by intent so each percentile describes one workload rather than a
  // mix of cheap FOLLOW cycles and expensive maneuver cycles.
  std::vector<PlannerCycleProfile> & window =
    profiling_windows_.at(static_cast<std::size_t>(sample.intent));
  window.push_back(std::move(sample));
  const std::size_t window_size = static_cast<std::size_t>(std::max(
      1, config_.profiling_log_every_n_cycles));
  if (window.size() < window_size) {
    return;
  }
  emitProfile(window.front().intent, window);
}

void PlannerDiagnostics::emitProfile(
  PlannerIntent intent,
  std::vector<PlannerCycleProfile> & window)
{
  struct Summary
  {
    double average;
    double p95;
    double maximum;
  };
  const auto summarize = [&](auto getter) {
      std::vector<double> values;
      values.reserve(window.size());
      double sum = 0.0;
      for (const auto & item : window) {
        const double value = getter(item);
        values.push_back(value);
        sum += value;
      }
      std::sort(values.begin(), values.end());
      const std::size_t p95_index = (95 * values.size() + 99) / 100 - 1;
      return Summary{sum / static_cast<double>(values.size()), values[p95_index], values.back()};
    };
  const auto format = [](const Summary & summary) {
      return std::array<double, 3>{summary.average, summary.p95, summary.maximum};
    };

  const auto cycle = format(summarize([](const auto & p) {return p.cycle_ms;}));
  const auto odom = format(summarize([](const auto & p) {return p.odom_conversion_ms;}));
  const auto state = format(summarize([](const auto & p) {return p.state_update_ms;}));
  const auto planner = format(summarize([](const auto & p) {return p.planner_ms;}));
  const auto decision_pub = format(summarize([](const auto & p) {return p.decision_publish_ms;}));
  const auto path_message = format(summarize([](const auto & p) {return p.path_message_ms;}));
  const auto tf = format(summarize([](const auto & p) {return p.tf_ms;}));
  const auto path_pub = format(summarize([](const auto & p) {return p.path_publish_ms;}));
  const auto marker_pub = format(summarize([](const auto & p) {return p.marker_publish_ms;}));
  const auto generation = format(summarize([](const auto & p) {
        return p.candidate_generation_ms;
  }));
  const auto collision = format(summarize([](const auto & p) {return p.collision_check_ms;}));
  const auto projection = format(summarize([](const auto & p) {
        return p.terminal_projection_ms;
  }));
  const auto velocity = format(summarize([](const auto & p) {return p.velocity_profile_ms;}));
  const auto selection = format(summarize([](const auto & p) {return p.selection_ms;}));
  const auto finalization = format(summarize([](const auto & p) {return p.finalization_ms;}));
  const auto candidates = summarize([](const auto & p) {
        return static_cast<double>(p.candidate_count);
  });
  const auto samples = summarize([](const auto & p) {
        return static_cast<double>(p.total_path_samples);
  });
  const auto max_samples = summarize([](const auto & p) {
        return static_cast<double>(p.max_path_samples);
  });
  const auto collision_poses = summarize([](const auto & p) {
        return static_cast<double>(p.collision_poses_checked);
  });
  const auto collision_rejected = summarize([](const auto & p) {
        return static_cast<double>(p.collision_rejected);
  });
  const auto velocity_rejected = summarize([](const auto & p) {
        return static_cast<double>(p.velocity_rejected);
  });
  const auto valid_candidates = summarize([](const auto & p) {
        return static_cast<double>(p.valid_candidate_count);
  });
  std::size_t ready_cycles = 0;
  std::size_t empty_pool_cycles = 0;
  std::size_t out_of_grid_cycles = 0;
  uint64_t hint_samples = 0;
  uint64_t hint_fallbacks = 0;
  std::size_t intent_changes = 0;
  std::size_t side_flips = 0;
  std::size_t no_path_cycles = 0;
  std::size_t steer_stale_cycles = 0;
  std::array<std::size_t, 4> mode_counts{};
  for (const auto & item : window) {
    ready_cycles += item.inputs_ready ? 1U : 0U;
    empty_pool_cycles += item.candidate_count == 0 ? 1U : 0U;
    out_of_grid_cycles += item.out_of_grid_rejected > 0 ? 1U : 0U;
    hint_samples += item.station_hint_samples;
    hint_fallbacks += item.station_hint_fallbacks;
    intent_changes += item.intent_changed ? 1U : 0U;
    side_flips += item.side_flipped ? 1U : 0U;
    no_path_cycles += item.path_published ? 0U : 1U;
    steer_stale_cycles += item.steering_fresh ? 0U : 1U;
    ++mode_counts.at(static_cast<std::size_t>(item.executed_mode));
  }
  const double hint_fallback_pct = hint_samples == 0 ? 0.0 :
    100.0 * static_cast<double>(hint_fallbacks) / static_cast<double>(hint_samples);

  const std::size_t grid_updates = grid_updates_since_report_;
  grid_updates_since_report_ = 0;
  std::array<double, 3> grid{0.0, 0.0, 0.0};
  if (!grid_profiling_window_.empty()) {
    double sum = 0.0;
    for (const double value : grid_profiling_window_) {
      sum += value;
    }
    std::sort(grid_profiling_window_.begin(), grid_profiling_window_.end());
    const std::size_t p95_index = (95 * grid_profiling_window_.size() + 99) / 100 - 1;
    grid = {sum / static_cast<double>(grid_profiling_window_.size()),
      grid_profiling_window_[p95_index], grid_profiling_window_.back()};
  }
  grid_profiling_window_.clear();
  const auto grid_cells = static_cast<std::size_t>(std::max(0, grid_width_)) *
    static_cast<std::size_t>(std::max(0, grid_height_));

  RCLCPP_INFO(logger_,
    "LOCAL_PLANNER_PROFILE intent=%s format=avg/p95/max window=%zu ready=%zu "
    "cycle_ms=%.3f/%.3f/%.3f odom_ms=%.3f/%.3f/%.3f state_ms=%.3f/%.3f/%.3f "
    "planner_ms=%.3f/%.3f/%.3f decision_pub_ms=%.3f/%.3f/%.3f "
    "path_msg_ms=%.3f/%.3f/%.3f tf_ms=%.3f/%.3f/%.3f path_pub_ms=%.3f/%.3f/%.3f "
    "marker_pub_ms=%.3f/%.3f/%.3f candidate_gen_ms=%.3f/%.3f/%.3f "
    "collision_ms=%.3f/%.3f/%.3f projection_ms=%.3f/%.3f/%.3f "
    "velocity_ms=%.3f/%.3f/%.3f selection_ms=%.3f/%.3f/%.3f "
    "finalization_ms=%.3f/%.3f/%.3f candidates=%.1f/%.1f/%.1f "
    "path_samples=%.1f/%.1f/%.1f max_path_samples=%.1f/%.1f/%.1f "
    "collision_poses=%.1f/%.1f/%.1f "
    "valid=%.1f/%.1f/%.1f collision_rej=%.1f/%.1f/%.1f velocity_rej=%.1f/%.1f/%.1f "
    "empty_pool_cycles=%zu out_of_grid_cycles=%zu "
    "station_hint_fallback=%llu/%llu(%.2f%%) "
    "intent_changes=%zu side_flips=%zu no_path_cycles=%zu steer_stale_cycles=%zu "
    "modes=none:%zu/maneuver:%zu/braking:%zu/unavailable:%zu "
    "grid_updates=%zu grid=%dx%d cells=%zu res=%.4f grid_ms=%.3f/%.3f/%.3f",
    intentToString(intent).c_str(), window.size(), ready_cycles,
    cycle[0], cycle[1], cycle[2], odom[0], odom[1], odom[2], state[0], state[1], state[2],
    planner[0], planner[1], planner[2], decision_pub[0], decision_pub[1], decision_pub[2],
    path_message[0], path_message[1], path_message[2], tf[0], tf[1], tf[2],
    path_pub[0], path_pub[1], path_pub[2], marker_pub[0], marker_pub[1], marker_pub[2],
    generation[0], generation[1], generation[2], collision[0], collision[1], collision[2],
    projection[0], projection[1], projection[2], velocity[0], velocity[1], velocity[2],
    selection[0], selection[1], selection[2], finalization[0], finalization[1], finalization[2],
    candidates.average, candidates.p95, candidates.maximum,
    samples.average, samples.p95, samples.maximum,
    max_samples.average, max_samples.p95, max_samples.maximum,
    collision_poses.average, collision_poses.p95, collision_poses.maximum,
    valid_candidates.average, valid_candidates.p95, valid_candidates.maximum,
    collision_rejected.average, collision_rejected.p95, collision_rejected.maximum,
    velocity_rejected.average, velocity_rejected.p95, velocity_rejected.maximum,
    empty_pool_cycles, out_of_grid_cycles,
    static_cast<unsigned long long>(hint_fallbacks),
    static_cast<unsigned long long>(hint_samples), hint_fallback_pct,
    intent_changes, side_flips, no_path_cycles, steer_stale_cycles,
    mode_counts[0], mode_counts[1], mode_counts[2], mode_counts[3],
    grid_updates, grid_width_, grid_height_, grid_cells, grid_resolution_,
    grid[0], grid[1], grid[2]);

  if (hint_fallback_pct > kStationHintFallbackWarnPct) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, kRareErrorThrottleMs,
      "Station-hint projection falling back %.1f%% of samples in %s: the cheap "
      "path is not being taken. Expect candidate_gen_ms up to ~60x. Check the "
      "raceline's curvature against horizon_m.",
      hint_fallback_pct, intentToString(intent).c_str());
  }
  window.clear();
}

void PlannerDiagnostics::noteTransitions(
  const PlannerDecisionData & data,
  bool path_published,
  bool steering_fresh,
  PlannerCycleProfile & sample)
{
  sample.path_published = path_published;
  sample.steering_fresh = steering_fresh;

  const bool first = !has_previous_cycle_;
  const bool intent_changed = !first && data.requested_intent != previous_intent_;
  const bool path_changed = !first && path_published != previous_path_published_;
  const bool steering_changed = !first && steering_fresh != previous_steering_fresh_;
  const bool side_flipped = !first && previous_terminal_d_m_ * data.terminal_d_m < 0.0;

  sample.intent_changed = intent_changed;
  sample.side_flipped = side_flipped;

  const PlannerIntent from_intent = previous_intent_;
  const bool from_path_published = previous_path_published_;
  const double from_terminal_d_m = previous_terminal_d_m_;
  const bool from_steering_fresh = previous_steering_fresh_;

  has_previous_cycle_ = true;
  previous_intent_ = data.requested_intent;
  previous_path_published_ = path_published;
  previous_terminal_d_m_ = data.terminal_d_m;
  previous_steering_fresh_ = steering_fresh;

  if (!config_.diagnostics_enabled ||
    (!intent_changed && !path_changed && !steering_changed && !side_flipped))
  {
    return;
  }

  const char * reason = data.opponent_detected ? "opponent" :
    (data.raceline_compatible ? "compatible" : "incompatible");

  RCLCPP_INFO(logger_,
    "LOCAL_PLANNER_EVENT %s->%s reason=%s "
    "ego_d=%+.3f(lim %.3f) head_err=%+.3f(lim %.3f) compatible=%d "
    "opp=%d gap=%+.2f rel=%s "
    "path=%s->%s term_d=%+.3f->%+.3f steer_fresh=%d->%d mode=%d "
    "track_ready=%d track_caps=R%.3f/L%.3f track_rej=%u "
    "valid=%u collision_rej=%u",
    intentToString(from_intent).c_str(), intentToString(data.requested_intent).c_str(),
    reason,
    data.ego_d_m, config_.vehicle_full_width_m,
    data.heading_error_rad, config_.compat_heading_rad,
    data.raceline_compatible ? 1 : 0,
    data.opponent_detected ? 1 : 0, data.opponent_gap_m,
    relativePositionToString(data.relative_position).c_str(),
    from_path_published ? "yes" : "no", path_published ? "yes" : "no",
    from_terminal_d_m, data.terminal_d_m,
    from_steering_fresh ? 1 : 0, steering_fresh ? 1 : 0,
    static_cast<int>(data.executed_mode),
    data.track_bounds_ready ? 1 : 0,
    data.sustainable_right_m, data.sustainable_left_m, data.track_bounds_rejected,
    data.valid_candidate_count, data.collision_rejected);
}

}  // namespace local_planning
