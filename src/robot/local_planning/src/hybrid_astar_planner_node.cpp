#include "planning/hybrid_astar_planner_node.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <fstream>
#include <sstream>
#include <thread>
#include <functional>
#include <algorithm>
#include <chrono>

namespace local_planning {

using namespace std::placeholders;

namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedMs(SteadyClock::time_point start, SteadyClock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

double elapsedMs(SteadyClock::time_point start)
{
    return elapsedMs(start, SteadyClock::now());
}

} // namespace

HybridAStarPlannerNode::HybridAStarPlannerNode()
    : Node("hybrid_astar_planner_node")
{
    // parameters check yaml for explanation
    this->declare_parameter<std::string>("racing_line_file", "racing_line.csv");
    this->declare_parameter<int>("default_intent", static_cast<int>(LocalPlannerIntent::FOLLOW_RACING_LINE));
    this->declare_parameter<double>("horizon_m", 6.0);
    this->declare_parameter<double>("layer_spacing_m", 0.5);
    this->declare_parameter<double>("lane_spacing_m", 0.1);
    this->declare_parameter<double>("max_lateral_offset_m", 1.8);
    this->declare_parameter<double>("max_path_angle_deg", 50.0);
    this->declare_parameter<double>("sample_spacing_m", 0.1);
    this->declare_parameter<double>("obstacle_inflation_distance_m", 0.2);
    this->declare_parameter<int>("occupied_threshold", 50);
    this->declare_parameter<double>("friction_coeff", 1.0);
    this->declare_parameter<double>("min_velocity_mps", 0.5);
    this->declare_parameter<double>("max_velocity_mps", 10.0);
    this->declare_parameter<double>("time_weight", 1.0);
    this->declare_parameter<double>("curvature_change_weight", 0.4);
    this->declare_parameter<double>("follow_d_weight", 0.20);
    this->declare_parameter<double>("overtake_d_weight", 0.02);
    this->declare_parameter<double>("merge_d_weight", 0.20);
    this->declare_parameter<double>("merge_terminal_d_weight", 0.0);
    this->declare_parameter<bool>("enable_runtime_diagnostics", true);
    this->declare_parameter<double>("planner_runtime_budget_ms", 100.0);
    this->declare_parameter<double>("diagnostics_log_period_ms", 1000.0);

    racing_line_file_ = this->get_parameter("racing_line_file").as_string();
    switch (std::clamp(static_cast<int>(this->get_parameter("default_intent").as_int()), 0, 2)) {
        case 1:
            default_intent_ = LocalPlannerIntent::OVERTAKE;
            break;
        case 2:
            default_intent_ = LocalPlannerIntent::MERGE;
            break;
        case 0:
        default:
            default_intent_ = LocalPlannerIntent::FOLLOW_RACING_LINE;
            break;
    }
    planner_config_.horizon_m = this->get_parameter("horizon_m").as_double();
    planner_config_.layer_spacing_m = this->get_parameter("layer_spacing_m").as_double();
    planner_config_.lane_spacing_m = this->get_parameter("lane_spacing_m").as_double();
    planner_config_.max_lateral_offset_m = this->get_parameter("max_lateral_offset_m").as_double();
    planner_config_.max_path_angle_deg = this->get_parameter("max_path_angle_deg").as_double();
    planner_config_.sample_spacing_m = this->get_parameter("sample_spacing_m").as_double();
    planner_config_.obstacle_inflation_distance_m = this->get_parameter("obstacle_inflation_distance_m").as_double();
    planner_config_.occupied_threshold = this->get_parameter("occupied_threshold").as_int();
    planner_config_.friction_coeff = this->get_parameter("friction_coeff").as_double();
    planner_config_.min_velocity_mps = this->get_parameter("min_velocity_mps").as_double();
    planner_config_.max_velocity_mps = this->get_parameter("max_velocity_mps").as_double();
    planner_config_.time_weight = this->get_parameter("time_weight").as_double();
    planner_config_.curvature_change_weight = this->get_parameter("curvature_change_weight").as_double();
    planner_config_.follow_d_weight = this->get_parameter("follow_d_weight").as_double();
    planner_config_.overtake_d_weight = this->get_parameter("overtake_d_weight").as_double();
    planner_config_.merge_d_weight = this->get_parameter("merge_d_weight").as_double();
    planner_config_.merge_terminal_d_weight = this->get_parameter("merge_terminal_d_weight").as_double();
    enable_runtime_diagnostics_ = this->get_parameter("enable_runtime_diagnostics").as_bool();
    planner_runtime_budget_ms_ = this->get_parameter("planner_runtime_budget_ms").as_double();
    diagnostics_log_period_ms_ = this->get_parameter("diagnostics_log_period_ms").as_double();

    // planner
    planner_ = std::make_unique<LocalFrenetLatticePlanner>();
    planner_->setConfig(planner_config_);

    // subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&HybridAStarPlannerNode::odometryCallback, this, _1));

    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid", 10,
        std::bind(&HybridAStarPlannerNode::occupancyGridCallback, this, _1));

    // publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    viz_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("/local_frenet_lattice_viz", 10);
    diagnostics_pub_ = this->create_publisher<local_planning::msg::PlannerDiagnostics>("/planner_diagnostics", 10);

    // action server
    plan_action_server_ = rclcpp_action::create_server<PlanPath>(
        this,
        "plan_path",
        std::bind(&HybridAStarPlannerNode::handleGoal, this, _1, _2),
        std::bind(&HybridAStarPlannerNode::handleCancel, this, _1),
        std::bind(&HybridAStarPlannerNode::handleAccepted, this, _1));

    loadRacingLine();

    RCLCPP_INFO(this->get_logger(), "Local Frenet lattice planner initialized");
}
//mutex this to prevent races
void HybridAStarPlannerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    current_odom_ = msg;
}

void HybridAStarPlannerNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    current_occupancy_grid_ = msg;
}

rclcpp_action::GoalResponse HybridAStarPlannerNode::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const PlanPath::Goal> /*goal*/)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HybridAStarPlannerNode::handleCancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void HybridAStarPlannerNode::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    const uint64_t sequence = ++latest_plan_sequence_;
    std::thread([this, goal_handle, sequence]() {
        executePlan(goal_handle, sequence);
    }).detach();
}

void HybridAStarPlannerNode::executePlan(const std::shared_ptr<GoalHandle> goal_handle, uint64_t sequence) {
    const auto total_start = SteadyClock::now();
    auto result = std::make_shared<PlanPath::Result>();

    auto goal = goal_handle->get_goal();
    const LocalPlannerIntent intent = intentFromAction(goal->intent);

    LocalFrenetPlan plan;
    plan.diagnostics.intent = intent;

    double input_copy_ms = 0.0;
    double ros_conversion_ms = 0.0;
    double planner_ms = 0.0;
    double path_conversion_ms = 0.0;
    double publish_ms = 0.0;
    double viz_ms = 0.0;

    auto emit_diagnostics = [&](bool success, bool stale_or_canceled, size_t path_points) {
        local_planning::msg::PlannerDiagnostics msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.sequence = sequence;
        msg.intent = static_cast<uint8_t>(intent);
        msg.success = success;
        msg.stale_or_canceled = stale_or_canceled;

        msg.total_ms = elapsedMs(total_start);
        msg.input_copy_ms = input_copy_ms;
        msg.ros_conversion_ms = ros_conversion_ms;
        msg.planner_ms = planner_ms;
        msg.path_conversion_ms = path_conversion_ms;
        msg.publish_ms = publish_ms;
        msg.viz_ms = viz_ms;

        const auto& diagnostics = plan.diagnostics;
        msg.planner_setup_ms = diagnostics.planner_setup_ms;
        msg.planner_search_ms = diagnostics.planner_search_ms;
        msg.planner_goal_select_ms = diagnostics.planner_goal_select_ms;
        msg.planner_reconstruct_ms = diagnostics.planner_reconstruct_ms;

        msg.layers = diagnostics.layers;
        msg.total_lanes = diagnostics.total_lanes;
        msg.attempted_edges = diagnostics.attempted_edges;
        msg.valid_edges = diagnostics.valid_edges;
        msg.invalid_collision_edges = diagnostics.invalid_collision_edges;
        msg.invalid_out_of_grid_edges = diagnostics.invalid_out_of_grid_edges;
        msg.invalid_geometry_edges = diagnostics.invalid_geometry_edges;
        msg.path_points = static_cast<uint32_t>(path_points);

        msg.budget_ms = planner_runtime_budget_ms_;
        msg.lane_spacing_m = planner_config_.lane_spacing_m;
        msg.layer_spacing_m = planner_config_.layer_spacing_m;
        msg.sample_spacing_m = planner_config_.sample_spacing_m;
        msg.horizon_m = planner_config_.horizon_m;
        msg.ego_s = diagnostics.ego_frenet.s;
        msg.ego_d = diagnostics.ego_frenet.d;
        msg.selected_cost = diagnostics.selected_cost;
        msg.selected_final_d = diagnostics.selected_final_d;

        publishRuntimeDiagnostics(msg);
    };

    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_msg;
    const auto input_copy_start = SteadyClock::now();
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        odom_msg = current_odom_;
        occupancy_grid_msg = current_occupancy_grid_;
    }
    input_copy_ms = elapsedMs(input_copy_start);

    if (!odom_msg || !occupancy_grid_msg) {
        RCLCPP_WARN(this->get_logger(), "Missing odom or occupancy grid, cannot plan");
        result->success = false;
        emit_diagnostics(false, false, 0);
        goal_handle->succeed(result);
        return;
    }

    if (goal_handle->is_canceling()) {
        result->success = false;
        emit_diagnostics(false, true, 0);
        goal_handle->canceled(result);
        return;
    }

    const auto conversion_start = SteadyClock::now();
    local_planning::Odometry odom = rosToOdometry(odom_msg);
    local_planning::OccupancyGrid grid = rosToOccupancyGrid(occupancy_grid_msg);
    ros_conversion_ms = elapsedMs(conversion_start);

    const auto planner_start = SteadyClock::now();
    plan = planner_->plan(odom.position, odom.heading, grid, intent);
    planner_ms = elapsedMs(planner_start);

    const auto path_conversion_start = SteadyClock::now();
    result->path = pathToRosPath(plan.path);
    path_conversion_ms = elapsedMs(path_conversion_start);

    const bool stale_plan = sequence != latest_plan_sequence_.load();
    if (stale_plan || goal_handle->is_canceling()) {
        result->success = false;
        emit_diagnostics(false, true, result->path.poses.size());
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
        return;
    }

    if (plan.path.empty()) {
        RCLCPP_WARN(this->get_logger(), "Planner returned empty path");
        result->success = false;
        std::lock_guard<std::mutex> lock(publish_mutex_);
        const bool stale_or_canceling = sequence != latest_plan_sequence_.load() || goal_handle->is_canceling();
        if (stale_or_canceling) {
            emit_diagnostics(false, true, result->path.poses.size());
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
            } else {
                goal_handle->succeed(result);
            }
            return;
        }
        const auto viz_start = SteadyClock::now();
        publishPlannerViz(plan);
        viz_ms = elapsedMs(viz_start);
        emit_diagnostics(false, false, result->path.poses.size());
        goal_handle->succeed(result);
        return;
    }

    result->success = true;

    std::lock_guard<std::mutex> lock(publish_mutex_);
    if (sequence != latest_plan_sequence_.load() || goal_handle->is_canceling()) {
        result->success = false;
        emit_diagnostics(false, true, result->path.poses.size());
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else {
            goal_handle->succeed(result);
        }
        return;
    }

    const auto publish_start = SteadyClock::now();
    path_pub_->publish(result->path);
    publish_ms = elapsedMs(publish_start);

    const auto viz_start = SteadyClock::now();
    publishPlannerViz(plan);
    viz_ms = elapsedMs(viz_start);

    emit_diagnostics(true, false, result->path.poses.size());
    goal_handle->succeed(result);
}

void HybridAStarPlannerNode::loadRacingLine() {
    std::ifstream file(racing_line_file_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open racing line file: %s", racing_line_file_.c_str());
        return;
    }

    racing_line_.clear();
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        double x, y, v;
        char comma;
        if (ss >> x >> comma >> y >> comma >> v) {
            racing_line_.emplace_back(x, y, v);
        }
    }

    if (racing_line_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Racing line is empty");
        return;
    }

    planner_->setRacingLine(racing_line_);
    viz_frenet_converter_.setRacingLine(racing_line_);
    RCLCPP_INFO(this->get_logger(), "Loaded racing line with %zu points", racing_line_.size());
}

LocalPlannerIntent HybridAStarPlannerNode::intentFromAction(uint8_t intent) const {
    switch (intent) {
        case PlanPath::Goal::OVERTAKE:
            return LocalPlannerIntent::OVERTAKE;
        case PlanPath::Goal::MERGE:
            return LocalPlannerIntent::MERGE;
        case PlanPath::Goal::FOLLOW_RACING_LINE:
            return LocalPlannerIntent::FOLLOW_RACING_LINE;
        default:
            return default_intent_;
    }
}

void HybridAStarPlannerNode::publishPlannerViz(const LocalFrenetPlan& plan) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = this->now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const LocalFrenetPlannerDiagnostics& diagnostics = plan.diagnostics;
    int marker_id = 0;

    for (double d : diagnostics.lane_offsets) {
        visualization_msgs::msg::Marker lane_marker;
        lane_marker.header.frame_id = "map";
        lane_marker.header.stamp = this->now();
        lane_marker.ns = "candidate_lanes";
        lane_marker.id = marker_id++;
        lane_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lane_marker.action = visualization_msgs::msg::Marker::ADD;
        lane_marker.scale.x = 0.015;
        lane_marker.color.r = 0.35;
        lane_marker.color.g = 0.55;
        lane_marker.color.b = 1.0;
        lane_marker.color.a = 0.28;

        for (int layer = 0; layer <= diagnostics.layers; ++layer) {
            Point p = viz_frenet_converter_.frenetToCartesian({
                diagnostics.ego_frenet.s + static_cast<double>(layer) * planner_config_.layer_spacing_m,
                d});
            geometry_msgs::msg::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.02;
            lane_marker.points.push_back(pt);
        }
        markers.markers.push_back(lane_marker);
    }

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = this->now();
    line_strip.ns = "selected_path";
    line_strip.id = marker_id++;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.08;
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.15;
    line_strip.color.a = 1.0;

    for (const auto& p : plan.path) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.0;
        line_strip.points.push_back(pt);
    }

    markers.markers.push_back(line_strip);
    viz_pub_->publish(markers);
}

void HybridAStarPlannerNode::publishRuntimeDiagnostics(
    const local_planning::msg::PlannerDiagnostics& diagnostics)
{
    if (!enable_runtime_diagnostics_) {
        return;
    }

    diagnostics_pub_->publish(diagnostics);

    const int64_t log_period_ms = std::max<int64_t>(
        1, static_cast<int64_t>(diagnostics_log_period_ms_));
    const char* status = diagnostics.success ? "success" : "failure";
    if (diagnostics.total_ms > planner_runtime_budget_ms_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), log_period_ms,
            "plan_ms=%.2f budget_ms=%.2f status=%s stale_or_canceled=%s lanes=%d layers=%d attempted_edges=%d valid_edges=%d path_points=%u planner_ms=%.2f search_ms=%.2f viz_ms=%.2f",
            diagnostics.total_ms,
            diagnostics.budget_ms,
            status,
            diagnostics.stale_or_canceled ? "true" : "false",
            diagnostics.total_lanes,
            diagnostics.layers,
            diagnostics.attempted_edges,
            diagnostics.valid_edges,
            diagnostics.path_points,
            diagnostics.planner_ms,
            diagnostics.planner_search_ms,
            diagnostics.viz_ms);
        return;
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), log_period_ms,
        "plan_ms=%.2f budget_ms=%.2f status=%s stale_or_canceled=%s lanes=%d layers=%d attempted_edges=%d valid_edges=%d path_points=%u planner_ms=%.2f search_ms=%.2f viz_ms=%.2f",
        diagnostics.total_ms,
        diagnostics.budget_ms,
        status,
        diagnostics.stale_or_canceled ? "true" : "false",
        diagnostics.total_lanes,
        diagnostics.layers,
        diagnostics.attempted_edges,
        diagnostics.valid_edges,
        diagnostics.path_points,
        diagnostics.planner_ms,
        diagnostics.planner_search_ms,
        diagnostics.viz_ms);
}

local_planning::Odometry HybridAStarPlannerNode::rosToOdometry(
    const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    local_planning::Odometry odom;
    odom.position.x = msg->pose.pose.position.x;
    odom.position.y = msg->pose.pose.position.y;
    odom.velocity = msg->twist.twist.linear.x;
    odom.heading = tf2::getYaw(msg->pose.pose.orientation);
    return odom;
}

local_planning::OccupancyGrid HybridAStarPlannerNode::rosToOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    local_planning::OccupancyGrid grid;
    grid.width = static_cast<int>(msg->info.width);
    grid.height = static_cast<int>(msg->info.height);
    grid.resolution = msg->info.resolution;
    grid.origin.x = msg->info.origin.position.x;
    grid.origin.y = msg->info.origin.position.y;
    grid.data.assign(msg->data.begin(), msg->data.end());
    return grid;
}

nav_msgs::msg::Path HybridAStarPlannerNode::pathToRosPath(
    const std::vector<local_planning::Point>& path)
{
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = "map";
    ros_path.header.stamp = this->now();

    for (const auto& p : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = p.velocity;
        pose.pose.orientation.w = 1.0;
        ros_path.poses.push_back(pose);
    }
    return ros_path;
}

} // namespace local_planning

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planning::HybridAStarPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
