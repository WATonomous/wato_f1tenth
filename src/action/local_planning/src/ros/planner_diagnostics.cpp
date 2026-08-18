#include "local_planning/ros/planner_diagnostics.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace local_planning
{

PlannerDiagnostics::PlannerDiagnostics(
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock,
  PlannerDiagnosticsConfig config)
: logger_(std::move(logger)), clock_(std::move(clock)), config_(std::move(config))
{
}

void PlannerDiagnostics::recordCycle(CycleProfile sample)
{
  if (sample.outcome.decision) {
    noteTransitions(sample);
  }
  recordProfile(std::move(sample));
}

void PlannerDiagnostics::recordProfile(CycleProfile sample)
{
  if (!config_.profiling_enabled) {
    return;
  }

  profiling_window_.push_back(std::move(sample));
  const std::size_t window_size = static_cast<std::size_t>(std::max(
      1, config_.profiling_log_every_n_cycles));
  if (profiling_window_.size() < window_size) {
    return;
  }
  emitProfile();
}

void PlannerDiagnostics::emitProfile()
{
  double sum = 0.0;
  for (const auto & item : profiling_window_) {
    sum += item.cycle_ms;
  }
  const double mean = sum / static_cast<double>(profiling_window_.size());
  RCLCPP_INFO(
    logger_, "LOCAL_PLANNER_PROFILE n=%zu cycle_ms=%.3f",
    profiling_window_.size(), mean);
  profiling_window_.clear();
}

void PlannerDiagnostics::noteTransitions(CycleProfile & sample)
{
  const PlannerDecisionData & data = *sample.outcome.decision;

  const bool first = !has_previous_cycle_;
  const bool intent_changed = !first && data.requested_intent != previous_intent_;
  const bool proposal_changed = !first &&
    data.proposed_intent != previous_proposed_intent_;
  const bool evidence_changed = !first &&
    (data.pending_grid_count != previous_pending_grid_count_ ||
    data.merge_probe_valid_cycles != previous_merge_probe_valid_cycles_);
  const bool path_changed = !first &&
    sample.outcome.path_published != previous_path_published_;
  const bool steering_changed = !first &&
    sample.outcome.steering_fresh != previous_steering_fresh_;
  const bool side_flipped = !first && previous_terminal_d_m_ * data.terminal_d_m < 0.0;

  sample.outcome.intent_changed = intent_changed;
  sample.outcome.side_flipped = side_flipped;

  const PlannerIntent from_intent = previous_intent_;
  const bool from_path_published = previous_path_published_;
  const double from_terminal_d_m = previous_terminal_d_m_;
  const bool from_steering_fresh = previous_steering_fresh_;

  has_previous_cycle_ = true;
  previous_intent_ = data.requested_intent;
  previous_proposed_intent_ = data.proposed_intent;
  previous_pending_grid_count_ = data.pending_grid_count;
  previous_merge_probe_valid_cycles_ = data.merge_probe_valid_cycles;
  previous_path_published_ = sample.outcome.path_published;
  previous_terminal_d_m_ = data.terminal_d_m;
  previous_steering_fresh_ = sample.outcome.steering_fresh;

  if (!config_.diagnostics_enabled ||
    (!intent_changed && !proposal_changed && !evidence_changed && !path_changed &&
    !steering_changed && !side_flipped))
  {
    return;
  }

  const char * reason = data.opponent_detected ? "opponent" :
    (data.raceline_compatible ? "compatible" : "incompatible");

  RCLCPP_INFO(logger_,
    "LOCAL_PLANNER_EVENT %s->%s proposed=%s executed=%s reason=%s "
    "transition_class=%u transition_reason=%u pending_s=%.3f pending_grids=%u "
    "costmap_seq=%llu costmap_stamp=%.6f costmap_age=%.3f opponent_seq=%llu "
    "merge_probe=%d probe_cycles=%u recovery=%u "
    "ego_d=%+.3f(lim %.3f) head_err=%+.3f(lim %.3f) compatible=%d "
    "opp=%d gap=%+.2f rel=%s "
    "path=%s->%s term_d=%+.3f->%+.3f steer_fresh=%d->%d mode=%d "
    "track_ready=%d track_caps=R%.3f/L%.3f track_rej=%u "
    "valid=%u collision_rej=%u",
    intentToString(from_intent).c_str(), intentToString(data.requested_intent).c_str(),
    intentToString(data.proposed_intent).c_str(), intentToString(data.executed_intent).c_str(),
    reason,
    static_cast<unsigned>(data.transition_class),
    static_cast<unsigned>(data.transition_reason), data.pending_transition_s,
    data.pending_grid_count,
    static_cast<unsigned long long>(data.costmap_sequence),
    data.costmap_stamp_s, clock_->now().seconds() - data.costmap_stamp_s,
    static_cast<unsigned long long>(data.opponent_observation_sequence),
    data.merge_probe_available ? 1 : 0, data.merge_probe_valid_cycles,
    static_cast<unsigned>(data.recovery_reason),
    data.ego_d_m, config_.vehicle_full_width_m,
    data.heading_error_rad, config_.compat_heading_rad,
    data.raceline_compatible ? 1 : 0,
    data.opponent_detected ? 1 : 0, data.opponent_gap_m,
    relativePositionToString(data.relative_position).c_str(),
    from_path_published ? "yes" : "no", sample.outcome.path_published ? "yes" : "no",
    from_terminal_d_m, data.terminal_d_m,
    from_steering_fresh ? 1 : 0, sample.outcome.steering_fresh ? 1 : 0,
    static_cast<int>(data.executed_mode),
    data.track_bounds_ready ? 1 : 0,
    data.sustainable_right_m, data.sustainable_left_m, data.track_bounds_rejected,
    data.valid_candidate_count, data.collision_rejected);
}

}  // namespace local_planning
