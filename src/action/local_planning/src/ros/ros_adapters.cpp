#include "local_planning/ros/ros_adapters.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace local_planning
{

Odometry rosToOdometry(const nav_msgs::msg::Odometry & msg)
{
  Odometry odom;
  odom.position.x = msg.pose.pose.position.x;
  odom.position.y = msg.pose.pose.position.y;
  odom.velocity = msg.twist.twist.linear.x;
  odom.heading = tf2::getYaw(msg.pose.pose.orientation);
  return odom;
}

OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg)
{
  OccupancyGrid grid;
  grid.width = static_cast<int>(msg.info.width);
  grid.height = static_cast<int>(msg.info.height);
  grid.resolution = msg.info.resolution;
  grid.origin.x = msg.info.origin.position.x;
  grid.origin.y = msg.info.origin.position.y;
  grid.data.assign(msg.data.begin(), msg.data.end());
  return grid;
}

nav_msgs::msg::Path pathToRos(
  const Path & path,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.poses.reserve(path.size());
  for (const auto & sample : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sample.x;
    pose.pose.position.y = sample.y;
    pose.pose.position.z = sample.speed;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  return msg;
}

msg::PlannerDecision plannerDecisionToRos(
  const PlannerDecisionData & data,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  msg::PlannerDecision out;
  out.header.stamp = stamp;
  out.header.frame_id = frame_id;
  out.requested_intent = static_cast<uint8_t>(data.requested_intent);
  out.relative_position = static_cast<uint8_t>(data.relative_position);
  out.opponent_detected = data.opponent_detected;
  out.opponent_gap_m = data.opponent_gap_m;
  out.ego_s_m = data.ego_s_m;
  out.ego_d_m = data.ego_d_m;
  out.heading_error_rad = data.heading_error_rad;
  out.raceline_compatible = data.raceline_compatible;
  out.executed_mode = static_cast<uint8_t>(data.executed_mode);
  out.candidate_source = static_cast<uint8_t>(data.candidate_source);
  out.selected_offset_tail = data.selected_offset_tail;
  out.selected_max_abs_d_m = data.selected_max_abs_d_m;
  out.projection_seed_was_stale = data.projection_seed_was_stale;
  out.projection_heading_check_relaxed = data.projection_heading_check_relaxed;
  out.clearance_class = static_cast<uint8_t>(data.clearance_class);
  out.minimum_clearance_m = data.minimum_clearance_m;
  out.max_abs_curvature_inv_m = data.max_abs_curvature_inv_m;
  out.min_speed_mps = data.min_speed_mps;
  out.max_speed_mps = data.max_speed_mps;
  out.start_curvature_inv_m = data.start_curvature_inv_m;
  out.start_curvature_from_steering = data.start_curvature_from_steering;
  out.terminal_d_m = data.terminal_d_m;
  out.best_cost_s = data.best_cost_s;
  out.median_cost_s = data.median_cost_s;
  out.generated_count = data.generated_count;
  out.collision_rejected = data.collision_rejected;
  out.out_of_grid_rejected = data.out_of_grid_rejected;
  out.velocity_rejected = data.velocity_rejected;
  out.track_bounds_ready = data.track_bounds_ready;
  out.sustainable_left_m = data.sustainable_left_m;
  out.sustainable_right_m = data.sustainable_right_m;
  out.track_bounds_rejected = data.track_bounds_rejected;
  out.valid_candidate_count = data.valid_candidate_count;
  out.cycle_time_ms = data.cycle_time_ms;
  return out;
}

} // namespace local_planning
