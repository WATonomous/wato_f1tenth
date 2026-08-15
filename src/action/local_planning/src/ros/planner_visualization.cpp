#include "local_planning/ros/planner_visualization.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cstdio>
#include <utility>

namespace local_planning
{

PlannerVisualization::PlannerVisualization(
  const RacelineReference & reference,
  PlannerVisualizationConfig config)
: reference_(reference), config_(std::move(config))
{
}

void PlannerVisualization::publishCandidates(
  const LocalPlanResult & result,
  const rclcpp::Time & stamp,
  MarkerPublisher & publisher) const
{
  MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.stamp = stamp;
  clear.header.frame_id = config_.map_frame;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (std::size_t i = 0; i < result.pool.size(); ++i) {
    visualization_msgs::msg::Marker line;
    line.header = clear.header;
    line.ns = "candidate_paths";
    line.id = static_cast<int>(i);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    const bool selected = static_cast<int>(i) == result.selected_index;
    line.scale.x = selected ? 0.08 : 0.025;
    line.color.a = selected ? 1.0F : 0.35F;
    line.color.g = selected ? 1.0F : 0.55F;
    line.color.b = selected ? 0.1F : 0.9F;
    line.points.reserve(result.pool[i].path.size());
    for (const auto & sample : result.pool[i].path) {
      geometry_msgs::msg::Point point;
      point.x = sample.x;
      point.y = sample.y;
      line.points.push_back(point);
    }
    markers.markers.push_back(std::move(line));
  }
  if (result.selected_index >= 0) {
    const auto & selected = result.pool.at(static_cast<std::size_t>(result.selected_index));
    if (!selected.path.empty()) {
      visualization_msgs::msg::Marker terminal;
      terminal.header = clear.header;
      terminal.ns = "selected_terminal_offset";
      terminal.id = 0;
      terminal.type = visualization_msgs::msg::Marker::SPHERE;
      terminal.action = visualization_msgs::msg::Marker::ADD;
      terminal.pose.position.x = selected.path.back().x;
      terminal.pose.position.y = selected.path.back().y;
      terminal.pose.orientation.w = 1.0;
      terminal.scale.x = 0.18;
      terminal.scale.y = 0.18;
      terminal.scale.z = 0.18;
      terminal.color.a = 1.0F;
      terminal.color.r = 1.0F;
      terminal.color.g = 0.75F;
      markers.markers.push_back(std::move(terminal));
    }
  }
  publisher.publish(markers);
}

void PlannerVisualization::publishProjection(
  const Odometry & odom,
  const TacticalState & state,
  const rclcpp::Time & stamp,
  MarkerPublisher & publisher) const
{
  if (!config_.publish_projection_markers || !reference_.valid()) {
    return;
  }

  const ReferenceGeometrySample foot = reference_.sampleAtS(state.ego_s);
  MarkerArray markers;
  visualization_msgs::msg::Marker prototype;
  prototype.header.stamp = stamp;
  prototype.header.frame_id = config_.map_frame;
  prototype.ns = "ego_projection";
  prototype.action = visualization_msgs::msg::Marker::ADD;
  prototype.pose.orientation.w = 1.0;
  prototype.color.a = 1.0F;
  prototype.color.r = state.raceline_compatible ? 0.15F : 1.0F;
  prototype.color.g = state.raceline_compatible ? 1.0F : 0.15F;
  prototype.color.b = 0.15F;

  auto projected = prototype;
  projected.id = 0;
  projected.type = visualization_msgs::msg::Marker::SPHERE;
  projected.pose.position.x = foot.x;
  projected.pose.position.y = foot.y;
  projected.scale.x = 0.22;
  projected.scale.y = 0.22;
  projected.scale.z = 0.22;
  markers.markers.push_back(std::move(projected));

  auto offset = prototype;
  offset.id = 1;
  offset.type = visualization_msgs::msg::Marker::LINE_STRIP;
  offset.scale.x = 0.04;
  geometry_msgs::msg::Point ego_point;
  ego_point.x = odom.position.x;
  ego_point.y = odom.position.y;
  geometry_msgs::msg::Point foot_point;
  foot_point.x = foot.x;
  foot_point.y = foot.y;
  offset.points.push_back(ego_point);
  offset.points.push_back(foot_point);
  markers.markers.push_back(std::move(offset));

  const auto arrow = [&prototype](
    int id, double x, double y, double heading, float red, float green, float blue) {
      auto marker = prototype;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      tf2::Quaternion rotation;
      rotation.setRPY(0.0, 0.0, heading);
      marker.pose.orientation = tf2::toMsg(rotation);
      marker.scale.x = 1.0;
      marker.scale.y = 0.06;
      marker.scale.z = 0.06;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      return marker;
    };
  markers.markers.push_back(arrow(2, foot.x, foot.y, foot.heading, 0.3F, 0.6F, 1.0F));
  markers.markers.push_back(
    arrow(3, odom.position.x, odom.position.y, odom.heading, 1.0F, 0.6F, 0.1F));

  auto text = prototype;
  text.id = 4;
  text.ns = "ego_projection_text";
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.pose.position.x = odom.position.x;
  text.pose.position.y = odom.position.y;
  text.pose.position.z = 0.6;
  text.scale.z = 0.22;
  std::array<char, 256> label{};
  std::snprintf(
    label.data(), label.size(),
    "%s%s\ns=%.2f/%.1f d=%+.2f/%.2f\nhead_err=%+.3f/%.3f",
    intentToString(state.intent).c_str(), state.ego_seed_was_stale ? " SEED-STALE" : "",
    state.ego_s, reference_.totalLength(),
    state.ego_d, config_.vehicle_full_width_m,
    state.heading_error_rad, config_.compat_heading_rad);
  text.text = label.data();
  markers.markers.push_back(std::move(text));

  publisher.publish(markers);
}

void PlannerVisualization::publishTrackBounds(
  const rclcpp::Time & stamp,
  MarkerPublisher & publisher) const
{
  if (!reference_.trackWidthsValid()) {
    return;
  }
  MarkerArray markers;
  const auto make_line = [&stamp, this](
    const char * name, int id, float red, float green, float blue)
    {
      visualization_msgs::msg::Marker line;
      line.header.stamp = stamp;
      line.header.frame_id = config_.map_frame;
      line.ns = name;
      line.id = id;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.025;
      line.color.a = 0.8F;
      line.color.r = red;
      line.color.g = green;
      line.color.b = blue;
      return line;
    };
  auto raw_left = make_line("track_bounds_raw", 0, 0.2F, 0.55F, 1.0F);
  auto raw_right = make_line("track_bounds_raw", 1, 0.2F, 0.55F, 1.0F);
  for (std::size_t i = 0; i <= reference_.widthSampleCount(); ++i) {
    const std::size_t index = i % reference_.widthSampleCount();
    const auto widths = reference_.widthSample(index);
    const auto reference = reference_.sampleAtS(widths.s);
    const auto append = [&reference](visualization_msgs::msg::Marker & marker, double d) {
        geometry_msgs::msg::Point point;
        point.x = reference.x + d * reference.normal_x;
        point.y = reference.y + d * reference.normal_y;
        marker.points.push_back(point);
      };
    append(raw_left, widths.raw.left_magnitude);
    append(raw_right, -widths.raw.right_magnitude);
  }
  markers.markers.push_back(std::move(raw_left));
  markers.markers.push_back(std::move(raw_right));
  publisher.publish(markers);
}

}  // namespace local_planning
