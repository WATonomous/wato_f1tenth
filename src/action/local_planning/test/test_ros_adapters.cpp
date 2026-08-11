#include "local_planning/ros/ros_adapters.hpp"

#include <gtest/gtest.h>

namespace local_planning
{

TEST(RosAdapters, PathPreservesGeometrySpeedAndHeader)
{
  Path path(2);
  path[0].x = 1.0;
  path[0].y = -2.0;
  path[0].speed = 3.5;
  path[1].x = 4.0;
  path[1].y = 5.0;
  path[1].speed = 6.5;
  const rclcpp::Time stamp(123, 456, RCL_ROS_TIME);

  const auto message = pathToRos(path, stamp, "map");

  ASSERT_EQ(message.poses.size(), 2U);
  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_EQ(rclcpp::Time(message.header.stamp, RCL_ROS_TIME), stamp);
  for (const auto & pose : message.poses) {
    EXPECT_EQ(pose.header, message.header);
    EXPECT_DOUBLE_EQ(pose.pose.orientation.w, 1.0);
  }
  EXPECT_DOUBLE_EQ(message.poses[0].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(message.poses[0].pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(message.poses[0].pose.position.z, 3.5);
  EXPECT_DOUBLE_EQ(message.poses[1].pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(message.poses[1].pose.position.y, 5.0);
  EXPECT_DOUBLE_EQ(message.poses[1].pose.position.z, 6.5);
}

TEST(RosAdapters, DecisionPreservesEveryField)
{
  PlannerDecisionData data;
  data.requested_intent = PlannerIntent::PASS;
  data.relative_position = RelativePosition::AHEAD_AND_CLEAR;
  data.opponent_detected = true;
  data.opponent_gap_m = 1.1;
  data.ego_s_m = 2.2;
  data.ego_d_m = -0.3;
  data.heading_error_rad = 0.4;
  data.raceline_compatible = true;
  data.executed_mode = ExecutedMode::MANEUVER;
  data.candidate_source = CandidateSource::PASS_RECOVERY;
  data.projection_seed_was_stale = true;
  data.clearance_class = CollisionStatus::SOFT_INFLATION;
  data.minimum_clearance_m = 0.5;
  data.max_abs_curvature_inv_m = 0.6;
  data.min_speed_mps = 0.7;
  data.max_speed_mps = 0.8;
  data.start_curvature_inv_m = 0.9;
  data.start_curvature_from_steering = true;
  data.terminal_d_m = -1.0;
  data.best_cost_s = 1.2;
  data.median_cost_s = 1.3;
  data.generated_count = 14;
  data.collision_rejected = 15;
  data.out_of_grid_rejected = 16;
  data.velocity_rejected = 17;
  data.track_bounds_ready = true;
  data.sustainable_left_m = 1.8;
  data.sustainable_right_m = 1.9;
  data.track_bounds_rejected = 20;
  data.valid_candidate_count = 21;
  data.cycle_time_ms = 2.2;
  const rclcpp::Time stamp(987, 654, RCL_ROS_TIME);

  const auto message = plannerDecisionToRos(data, stamp, "world");

  EXPECT_EQ(message.header.frame_id, "world");
  EXPECT_EQ(rclcpp::Time(message.header.stamp, RCL_ROS_TIME), stamp);
  EXPECT_EQ(message.requested_intent, static_cast<uint8_t>(data.requested_intent));
  EXPECT_EQ(message.relative_position, static_cast<uint8_t>(data.relative_position));
  EXPECT_EQ(message.opponent_detected, data.opponent_detected);
  EXPECT_DOUBLE_EQ(message.opponent_gap_m, data.opponent_gap_m);
  EXPECT_DOUBLE_EQ(message.ego_s_m, data.ego_s_m);
  EXPECT_DOUBLE_EQ(message.ego_d_m, data.ego_d_m);
  EXPECT_DOUBLE_EQ(message.heading_error_rad, data.heading_error_rad);
  EXPECT_EQ(message.raceline_compatible, data.raceline_compatible);
  EXPECT_EQ(message.executed_mode, static_cast<uint8_t>(data.executed_mode));
  EXPECT_EQ(message.candidate_source, static_cast<uint8_t>(data.candidate_source));
  EXPECT_EQ(message.projection_seed_was_stale, data.projection_seed_was_stale);
  EXPECT_EQ(message.clearance_class, static_cast<uint8_t>(data.clearance_class));
  EXPECT_DOUBLE_EQ(message.minimum_clearance_m, data.minimum_clearance_m);
  EXPECT_DOUBLE_EQ(message.max_abs_curvature_inv_m, data.max_abs_curvature_inv_m);
  EXPECT_DOUBLE_EQ(message.min_speed_mps, data.min_speed_mps);
  EXPECT_DOUBLE_EQ(message.max_speed_mps, data.max_speed_mps);
  EXPECT_DOUBLE_EQ(message.start_curvature_inv_m, data.start_curvature_inv_m);
  EXPECT_EQ(message.start_curvature_from_steering, data.start_curvature_from_steering);
  EXPECT_DOUBLE_EQ(message.terminal_d_m, data.terminal_d_m);
  EXPECT_DOUBLE_EQ(message.best_cost_s, data.best_cost_s);
  EXPECT_DOUBLE_EQ(message.median_cost_s, data.median_cost_s);
  EXPECT_EQ(message.generated_count, data.generated_count);
  EXPECT_EQ(message.collision_rejected, data.collision_rejected);
  EXPECT_EQ(message.out_of_grid_rejected, data.out_of_grid_rejected);
  EXPECT_EQ(message.velocity_rejected, data.velocity_rejected);
  EXPECT_EQ(message.track_bounds_ready, data.track_bounds_ready);
  EXPECT_DOUBLE_EQ(message.sustainable_left_m, data.sustainable_left_m);
  EXPECT_DOUBLE_EQ(message.sustainable_right_m, data.sustainable_right_m);
  EXPECT_EQ(message.track_bounds_rejected, data.track_bounds_rejected);
  EXPECT_EQ(message.valid_candidate_count, data.valid_candidate_count);
  EXPECT_DOUBLE_EQ(message.cycle_time_ms, data.cycle_time_ms);
}

}  // namespace local_planning
