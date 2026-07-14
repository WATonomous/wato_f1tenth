#ifndef PLANNING_PLANNER_PLANNER_NODE_HPP
#define PLANNING_PLANNER_PLANNER_NODE_HPP

#include "planning/planner/local_planner.hpp"
#include "planning/types.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <local_planning/msg/planner_intent.hpp>
#include <local_planning/msg/planner_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace local_planning
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();
  ~PlannerNode() override;

private:
  void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void steeringCommandCallback(ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void occupancyGridCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void racingLineCallback(nav_msgs::msg::Path::SharedPtr msg);
  void intentCallback(msg::PlannerIntent::SharedPtr msg);
  void workerLoop();
  void runAttempt(std::chrono::steady_clock::time_point start);
  void publishStatus(
    uint8_t status, double runtime_ms, uint32_t path_points,
    const nav_msgs::msg::Odometry::SharedPtr & odom,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
    const nav_msgs::msg::Path::SharedPtr & reference,
    LocalPlannerIntent intent);
  void publishPlannerViz(const LocalFrenetPlan & plan);
  nav_msgs::msg::Path pathToRosPath(
    const std::vector<Point> & path,
    const std_msgs::msg::Header & source_header);
  bool transformPathToControllerFrame(
    const nav_msgs::msg::Path & planner_path,
    nav_msgs::msg::Path & controller_path);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr steering_command_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr racing_line_sub_;
  rclcpp::Subscription<msg::PlannerIntent>::SharedPtr intent_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<msg::PlannerStatus>::SharedPtr status_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<LocalPlanner> planner_;

  std::mutex input_mutex_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  std::optional<double> current_steering_command_;
  std::chrono::steady_clock::time_point current_steering_command_received_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_msg_;
  std::shared_ptr<const OccupancyGrid> current_grid_;
  nav_msgs::msg::Path::SharedPtr current_reference_msg_;
  std::shared_ptr<const std::vector<Point>> current_reference_;
  LocalPlannerIntent current_intent_ = LocalPlannerIntent::FOLLOW_RACING_LINE;
  bool has_intent_ = false;
  std::shared_ptr<const std::vector<Point>> planner_reference_guard_;

  std::atomic_bool stop_worker_{false};
  std::mutex sleep_mutex_;
  std::condition_variable sleep_cv_;
  std::thread worker_;
  uint64_t completed_attempts_ = 0;
  uint64_t overruns_ = 0;
  builtin_interfaces::msg::Time last_trajectory_stamp_;

  std::string racing_line_topic_;
  std::string steering_command_topic_;
  std::string planner_path_frame_;
  std::string controller_path_frame_;
  std::string debug_path_topic_;
  LocalFrenetPlannerConfig planner_config_;
  std::chrono::steady_clock::duration worker_period_;
};

}  // namespace local_planning

#endif
