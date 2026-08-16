#ifndef LOCAL_PLANNING_ROS_PLANNER_NODE_HPP
#define LOCAL_PLANNING_ROS_PLANNER_NODE_HPP

#include "local_planning/curves/frenet_connection_generator.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"
#include "local_planning/msg/planner_decision.hpp"
#include "local_planning/planning/local_planner.hpp"
#include "local_planning/reference/raceline_reference.hpp"
#include "local_planning/ros/planner_diagnostics.hpp"
#include "local_planning/ros/planner_visualization.hpp"
#include "local_planning/state/racing_state_machine.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <global_planner/msg/reference_track.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <string>

namespace local_planning
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  struct NodeConfig
  {
    VehicleGeometry vehicle_geometry;
    GridPolicy grid_policy;
    ManeuverConfig maneuver;
    CollisionConfig collision;
    VelocityProfileConfig velocity;
    FrenetConnectionConfig curve;
    ProjectionConfig projection;
    StateMachineConfig state;
    double planner_rate_hz = 20.0;
    double steering_command_timeout_s = 0.06;
    double odom_timeout_s = 0.25;
    double wheelbase_m = 0.33;
    double width_lookup_spacing_m = 0.10;
    bool use_steering_start_curvature = true;
    bool profiling_enabled = true;
    int profiling_log_every_n_cycles = 20;
    // Per-transition event lines.  Independent of profiling_enabled: profiling
    // answers "how long", this answers "what changed", and when the car twitches
    // the second question is the one that matters.
    bool diagnostics_enabled = true;
    // Empty means report every intent on its own line.  List any of
    // FOLLOW_RACING_LINE/OVERTAKE/PASS/MERGE to log only those, which is what
    // you want when only the maneuvering states matter.
    std::vector<PlannerIntent> profiling_intent_filter;
    std::string reference_track_topic;
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
    std::string track_bounds_visualization_topic;
    std::string projection_visualization_topic;
    bool publish_projection_markers = true;
  };

  NodeConfig loadConfig();
  void planningCycle();
  std::optional<Odometry> odometryInMap();
  bool transformPathToControllerFrame(
    const nav_msgs::msg::Path & map_path,
    nav_msgs::msg::Path & controller_path);
  void publishDecision(const PlannerDecisionData & data);
  void publishOvertakeReady(bool ready);

  NodeConfig config_;
  RacelineReference reference_;
  FrenetConnectionGenerator curve_generator_;
  ManeuverBuilder maneuver_builder_;
  RacingStateMachine state_machine_;
  LocalPlanner planner_;
  PlannerDiagnostics diagnostics_;
  PlannerVisualization visualization_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  OccupancyGrid grid_;
  bool has_grid_ = false;
  double steering_angle_ = 0.0;
  // ROS clock, not steady_clock: under use_sim_time the two are unrelated, and
  // steering_command_timeout_s is a budget in simulated seconds.
  rclcpp::Time steering_received_;
  bool has_steering_ = false;
  std::optional<bool> last_overtake_ready_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<global_planner::msg::ReferenceTrack>::SharedPtr reference_track_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr steering_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overtake_ready_pub_;
  rclcpp::Publisher<msg::PlannerDecision>::SharedPtr decision_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    track_bounds_visualization_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    projection_visualization_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_ROS_PLANNER_NODE_HPP
