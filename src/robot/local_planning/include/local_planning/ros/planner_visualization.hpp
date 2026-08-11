#ifndef LOCAL_PLANNING_ROS_PLANNER_VISUALIZATION_HPP
#define LOCAL_PLANNING_ROS_PLANNER_VISUALIZATION_HPP

#include "local_planning/planning/local_planner.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "local_planning/state/racing_state_machine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

namespace local_planning
{

struct PlannerVisualizationConfig
{
  std::string map_frame;
  bool publish_projection_markers = true;
  double vehicle_full_width_m = 0.0;
  double compat_heading_rad = 0.0;
};

class PlannerVisualization
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using MarkerPublisher = rclcpp::Publisher<MarkerArray>;

  PlannerVisualization(
    const RacelineReference & reference,
    PlannerVisualizationConfig config);

  void publishCandidates(
    const LocalPlanResult & result,
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;
  void publishProjection(
    const Odometry & odom,
    const TacticalState & state,
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;
  void publishTrackBounds(
    const rclcpp::Time & stamp,
    MarkerPublisher & publisher) const;

private:
  const RacelineReference & reference_;
  const PlannerVisualizationConfig config_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_ROS_PLANNER_VISUALIZATION_HPP
