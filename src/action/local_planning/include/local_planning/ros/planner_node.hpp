#ifndef LOCAL_PLANNING_ROS_PLANNER_NODE_HPP
#define LOCAL_PLANNING_ROS_PLANNER_NODE_HPP

#include "local_planning/curves/curve_connection_generator.hpp"
#include "local_planning/maneuvers/maneuver_builder.hpp"
#include "local_planning/msg/planner_decision.hpp"
#include "local_planning/planning/local_planner.hpp"
#include "local_planning/reference/raceline_reference.hpp"
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

#include <array>
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
    VehicleGeometry vehicle_geometry;
    GridPolicy grid_policy;
    ManeuverConfig maneuver;
    CollisionConfig collision;
    VelocityProfileConfig velocity;
    CurveGeneratorConfig curve;
    ProjectionConfig projection;
    StateMachineConfig state;
    double planner_rate_hz = 20.0;
    double steering_command_timeout_s = 0.06;
    double wheelbase_m = 0.33;
    double track_boundary_margin_m = 0.05;
    double width_lookup_spacing_m = 0.10;
    bool use_steering_start_curvature = true;
    bool profiling_enabled = true;
    int profiling_log_every_n_cycles = 20;
    // Per-transition event lines.  Independent of profiling_enabled: profiling
    // answers "how long", this answers "what changed", and when the car twitches
    // the second question is the one that matters.
    bool diagnostics_enabled = true;
    // Empty means report every intent on its own line.  Set to one of
    // FOLLOW_RACING_LINE/OVERTAKE/PASS/MERGE to log only that intent, which is
    // what you want when only the expensive state matters.
    std::optional<PlannerIntent> profiling_intent_filter;
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
  nav_msgs::msg::Path pathMessage(const Path & path) const;
  bool transformPathToControllerFrame(
    const nav_msgs::msg::Path & map_path,
    nav_msgs::msg::Path & controller_path);
  void publishDecision(const PlannerDecisionData & data);
  void publishOvertakeReady(bool ready);
  void publishMarkers(const LocalPlanResult & result);
  void publishTrackBoundsMarkers();
  // Draws the ego projection the FOLLOW/MERGE gate actually used: the foot on
  // the reference, the offset that becomes ego_d, and the two headings whose
  // difference becomes heading_error_rad.  Published every cycle, on its own
  // topic, because the intent this explains is usually one where publishMarkers
  // never runs.
  void publishProjectionMarkers(const Odometry & odom, const TacticalState & state);

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
    // Why candidates died, and what the cycle actually executed.  Without these
    // a zero-candidate cycle and an everything-collided cycle look identical in
    // the aggregate, and they call for opposite fixes.
    uint32_t collision_rejected = 0;
    uint32_t out_of_grid_rejected = 0;
    uint32_t velocity_rejected = 0;
    uint32_t valid_candidate_count = 0;
    // Slow-path rate in the side/deviation sweep.  Reported because the fast and
    // slow paths produce identical paths and differ only in cost.
    uint64_t station_hint_samples = 0;
    uint64_t station_hint_fallbacks = 0;
    // Per-cycle transition flags.  A jerk is a discontinuity, so what matters is
    // how often the cycle-to-cycle answer changed, not what any one cycle said.
    bool path_published = false;
    bool steering_fresh = false;
    bool intent_changed = false;
    bool side_flipped = false;
    ExecutedMode executed_mode = ExecutedMode::NO_LOCAL_PATH;
    bool inputs_ready = false;
    // Which intent produced this cycle's workload.  Cycles are aggregated per
    // intent, never pooled: see recordProfile().
    PlannerIntent intent = PlannerIntent::FOLLOW_RACING_LINE;
  };

  void recordProfile(ProfileSample sample);
  void emitProfile(PlannerIntent intent, std::vector<ProfileSample> & window);

  // Compares this cycle against the last one, flags the differences on sample,
  // and logs a line when anything the controller can feel changed.
  void noteTransitions(
    const PlannerDecisionData & data,
    bool path_published,
    bool steering_fresh,
    ProfileSample & sample);

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
  // ROS clock, not steady_clock: under use_sim_time the two are unrelated, and
  // steering_command_timeout_s is a budget in simulated seconds.
  rclcpp::Time steering_received_;
  bool has_steering_ = false;
  std::optional<bool> last_overtake_ready_;
  uint64_t profiling_cycle_count_ = 0;
  // One window per PlannerIntent, indexed by the enum value.  FOLLOW runs about
  // 1 ms and OVERTAKE tens of ms, so a pooled window reports percentiles that
  // track how many overtakes happened to land in it rather than what an
  // overtake actually costs.
  std::array<std::vector<ProfileSample>, 4> profiling_windows_;
  // Grid callbacks run far faster than the planner timer, so their timings are
  // accumulated here and reported inside the periodic profile line.
  std::vector<double> grid_profiling_window_;
  // Counted separately from the window above, which is capped: with an intent
  // filter the window saturates and would understate the real update count.
  std::size_t grid_updates_since_report_ = 0;
  // Last cycle's answers, for the transition comparison.  Nothing reads these
  // to plan with; they exist so a discontinuity has something to be measured
  // against.
  bool has_previous_cycle_ = false;
  PlannerIntent previous_intent_ = PlannerIntent::FOLLOW_RACING_LINE;
  bool previous_path_published_ = false;
  double previous_terminal_d_m_ = 0.0;
  bool previous_steering_fresh_ = false;

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
