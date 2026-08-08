#ifndef LOCAL_PLANNING_ROS_PLANNER_NODE_HPP
#define LOCAL_PLANNING_ROS_PLANNER_NODE_HPP

#include "local_planning/curves/curve_connection_generator.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"
#include "local_planning/msg/planner_decision.hpp"
#include "local_planning/planning/local_planner.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "local_planning/state/racing_state_machine.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace local_planning
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  struct NodeConfig
  {
    ManeuverConfig maneuver;
    LocalPlannerConfig planner;
    CurveGeneratorConfig curve;
    ProjectionConfig projection;
    StateMachineConfig state;
    double planner_rate_hz = 20.0;
    double steering_command_timeout_s = 0.06;
    double wheelbase_m = 0.33;
    bool use_steering_start_curvature = true;
    bool profiling_enabled = true;
    int profiling_log_every_n_cycles = 20;
    std::string racing_line_topic;
    std::string occupancy_grid_topic;
    std::string odom_topic;
    std::string steering_command_topic;
    std::string map_frame;
    std::string controller_frame;
    std::string local_path_topic;
    std::string local_path_map_topic;
    std::string overtake_ready_topic;
    std::string decision_topic;
    std::string visualization_topic;
  };

  NodeConfig loadConfig();
  void planningCycle();
  nav_msgs::msg::Path pathMessage(const Path & path) const;
  bool transformPathToControllerFrame(
    const nav_msgs::msg::Path & map_path,
    nav_msgs::msg::Path & controller_path);
  void publishDecision(const PlannerDecisionData & data);
  void publishOvertakeReady(bool ready);
  void publishMarkers(const LocalPlanResult & result);

  struct ProfileSample
  {
    double cycle_ms = 0.0;
    double odom_conversion_ms = 0.0;
    double state_update_ms = 0.0;
    double planner_ms = 0.0;
    double decision_publish_ms = 0.0;
    double path_message_ms = 0.0;
    double tf_ms = 0.0;
    double path_publish_ms = 0.0;
    double marker_publish_ms = 0.0;
    double candidate_generation_ms = 0.0;
    double collision_check_ms = 0.0;
    double terminal_projection_ms = 0.0;
    double velocity_profile_ms = 0.0;
    double selection_ms = 0.0;
    double finalization_ms = 0.0;
    uint32_t candidate_count = 0;
    uint32_t total_path_samples = 0;
    uint32_t max_path_samples = 0;
    uint32_t collision_poses_checked = 0;
    bool inputs_ready = false;
  };

  void recordProfile(ProfileSample sample);

  NodeConfig config_;
  RacelineReference reference_;
  CurveConnectionGenerator curve_generator_;
  ManeuverBuilder maneuver_builder_;
  RacingStateMachine state_machine_;
  LocalPlanner planner_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  OccupancyGrid grid_;
  bool has_grid_ = false;
  double steering_angle_ = 0.0;
  std::chrono::steady_clock::time_point steering_received_;
  bool has_steering_ = false;
  std::optional<bool> last_overtake_ready_;
  uint64_t profiling_cycle_count_ = 0;
  std::vector<ProfileSample> profiling_window_;
  // Grid callbacks run far faster than the planner timer, so their timings are
  // accumulated here and reported inside the periodic profile line.
  std::vector<double> grid_profiling_window_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr racing_line_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr steering_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overtake_ready_pub_;
  rclcpp::Publisher<msg::PlannerDecision>::SharedPtr decision_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_ROS_PLANNER_NODE_HPP
