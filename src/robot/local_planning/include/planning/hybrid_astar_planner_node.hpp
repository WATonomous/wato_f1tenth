#ifndef PLANNING_HYBRID_ASTAR_PLANNER_NODE_HPP
#define PLANNING_HYBRID_ASTAR_PLANNER_NODE_HPP

#include "local_frenet_lattice_planner.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <local_planning/action/plan_path.hpp>

#include <memory>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>
#include <string>

namespace local_planning {

class HybridAStarPlannerNode : public rclcpp::Node {
public:
    HybridAStarPlannerNode();
    ~HybridAStarPlannerNode() = default;

private:
    using PlanPath = local_planning::action::PlanPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<PlanPath>;

    // action server callbacks
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PlanPath::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

    void executePlan(const std::shared_ptr<GoalHandle> goal_handle, uint64_t sequence);

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // utility
    void loadRacingLine();
    void publishPlannerViz(const LocalFrenetPlan& plan);
    LocalPlannerIntent intentFromAction(uint8_t intent) const;

    // conversions
    local_planning::Odometry rosToOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);
    local_planning::OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);
    nav_msgs::msg::Path pathToRosPath(const std::vector<local_planning::Point>& path);

    // action server
    rclcpp_action::Server<PlanPath>::SharedPtr plan_action_server_;

    // subs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    // pubs
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

    // planner
    std::unique_ptr<LocalFrenetLatticePlanner> planner_;

    // cached messages
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_occupancy_grid_;
    std::mutex input_mutex_;
    std::mutex publish_mutex_;
    std::atomic<uint64_t> latest_plan_sequence_{0};

    // racing line
    std::vector<local_planning::Point> racing_line_;
    FrenetConverter viz_frenet_converter_;

    // parameters
    std::string racing_line_file_;
    LocalFrenetPlannerConfig planner_config_;
    LocalPlannerIntent default_intent_;
};

} // namespace local_planning

#endif // PLANNING_HYBRID_ASTAR_PLANNER_NODE_HPP
