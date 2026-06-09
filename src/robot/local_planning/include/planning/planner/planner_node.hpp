#ifndef PLANNING_PLANNER_PLANNER_NODE_HPP
#define PLANNING_PLANNER_PLANNER_NODE_HPP

#include "planning/planner/local_planner.hpp"
#include "planning/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <local_planning/action/plan_path.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace local_planning
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();
  ~PlannerNode() = default;

private:
  using PlanPath = local_planning::action::PlanPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanPath>;

  // action server callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanPath::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void executePlan(const std::shared_ptr<GoalHandle> goal_handle, uint64_t sequence);

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // utility
  void loadRacingLine();
  void publishPlannerViz(const LocalFrenetPlan & plan);
  LocalPlannerIntent intentFromAction(uint8_t intent) const;

  // conversions
  local_planning::Odometry rosToOdometry(const nav_msgs::msg::Odometry::SharedPtr & msg);
  local_planning::OccupancyGrid rosToOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & msg);
  void buildClearanceMasks(local_planning::OccupancyGrid & grid) const;
  nav_msgs::msg::Path pathToRosPath(
    const std::vector<local_planning::Point> & path, const std::string & frame_id);
  bool transformPathToControllerFrame(
    const nav_msgs::msg::Path & planner_path, nav_msgs::msg::Path & controller_path);

  // action server
  rclcpp_action::Server<PlanPath>::SharedPtr plan_action_server_;

  // subs
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  // pubs
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // planner
  std::unique_ptr<LocalPlanner> planner_;

  // cached messages
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  local_planning::OccupancyGrid current_occupancy_grid_;
  bool has_current_occupancy_grid_ = false;
  std::mutex input_mutex_;
  std::mutex publish_mutex_;
  std::atomic<uint64_t> latest_plan_sequence_{0};
  std::atomic_bool planner_busy_{false};

  // racing line
  std::vector<local_planning::Point> racing_line_;

  // parameters
  std::string racing_line_file_;
  LocalFrenetPlannerConfig planner_config_;
  LocalPlannerIntent default_intent_;
  double planner_runtime_budget_ms_ = 100.0;
  std::string planner_path_frame_;
  std::string controller_path_frame_;
  std::string debug_path_topic_;
};

} // namespace local_planning

#endif // PLANNING_PLANNER_PLANNER_NODE_HPP
