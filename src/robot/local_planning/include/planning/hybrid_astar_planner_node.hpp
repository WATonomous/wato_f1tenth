#ifndef PLANNING_HYBRID_ASTAR_PLANNER_NODE_HPP
#define PLANNING_HYBRID_ASTAR_PLANNER_NODE_HPP

#include "hybrid_astar_planner_code.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <local_planning/action/plan_path.hpp>

#include <memory>
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

    void executePlan(const std::shared_ptr<GoalHandle> goal_handle);

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // utility
    void loadRacingLine();
    void publishPathViz(const std::vector<local_planning::Point>& path);

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
    std::unique_ptr<HybridAStarPlanner> planner_;

    // cached messages
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_occupancy_grid_;

    // racing line
    std::vector<local_planning::Point> racing_line_;

    // parameters
    std::string racing_line_file_;
    double lane_spacing_;
    double layer_depth_;
    double waypoint_spacing_;
    double collision_check_spacing_;
    double friction_coeff_;
};

} // namespace local_planning

#endif // PLANNING_HYBRID_ASTAR_PLANNER_NODE_HPP
