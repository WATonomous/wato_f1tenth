#include "local_planner/local_planner_core.hpp"

#include <algorithm>
#include <cmath>

LocalPlannerCore::LocalPlannerCore(const CoreParams &params)
    : params_(params),
      checker_(params.planning_distance, params.max_lateral, params.obstacle_inflation),
      rrt_(params, checker_),
      rng_(std::random_device{}()) {}

PlanResult LocalPlannerCore::plan(
    const geometry_msgs::msg::Point &goal_local,
    const ScanFrame &scan,
    const std::vector<geometry_msgs::msg::Point> &manual_obstacles) {
    last_debug_ = PlanDebug{};
    geometry_msgs::msg::Point goal = goal_local;
    goal.x = std::clamp(goal.x, params_.step_size, params_.planning_distance);
    goal.y = std::clamp(goal.y, -params_.max_lateral, params_.max_lateral);
    goal.z = (std::isfinite(goal.z) && goal.z > 0.0) ? goal.z : params_.path_speed;

    auto obstacles = extract_obstacles(scan);
    obstacles.insert(obstacles.end(), manual_obstacles.begin(), manual_obstacles.end());
    last_debug_.obstacles = obstacles;

    std::optional<LocalPath> path;
    if (params_.use_straight_path) {
        path = build_straight_path(goal, obstacles);
        last_debug_.used_straight = path.has_value();
    }
    if (!path.has_value()) {
        path = rrt_.plan(goal, obstacles, rng_);
        last_debug_.used_rrt = path.has_value();
        last_debug_.rrt = rrt_.last_debug();
    }

    if (!path.has_value()) {
        return {PlanStatus::NO_PATH_FOUND, std::nullopt};
    }

    return {PlanStatus::OK, std::move(path)};
}

std::vector<geometry_msgs::msg::Point> LocalPlannerCore::extract_obstacles(
    const ScanFrame &scan) const {
    std::vector<geometry_msgs::msg::Point> obstacles;
    if (scan.ranges.empty()) {
        return obstacles;
    }

    double angle = scan.angle_min;
    for (const auto range : scan.ranges) {
        if (std::isfinite(range) && range >= params_.scan_min_range && range <= params_.scan_max_range) {
            geometry_msgs::msg::Point obstacle;
            obstacle.x = params_.laser_to_base_x + range * std::cos(angle);
            obstacle.y = params_.laser_to_base_y + range * std::sin(angle);

            if (obstacle.x >= params_.scan_front_min_x &&
                obstacle.x < params_.planning_distance &&
                std::abs(obstacle.y) <= params_.max_lateral) {
                obstacles.push_back(obstacle);
            }
        }
        angle += scan.angle_increment;
    }
    
    return obstacles;
}

std::optional<LocalPath> LocalPlannerCore::build_straight_path(
    const geometry_msgs::msg::Point &goal,
    const std::vector<geometry_msgs::msg::Point> &obstacles) const {
    if (!checker_.segment_is_free(0.0, 0.0, goal.x, goal.y, obstacles)) {
        return std::nullopt;
    }

    LocalPath path;
    const int points = std::max(2, params_.plan_points);
    for (int i = 0; i < points; ++i) {
        const double t = static_cast<double>(i) / (points - 1);
        const double x = t * goal.x;
        const double y = t * goal.y;
        const double clearance = checker_.point_clearance(x, y, obstacles);
        const double speed_scale = std::clamp(
            clearance / std::max(checker_.obstacle_inflation(), 1e-6), 0.5, 1.0);
        path.poses.push_back(make_pose(x, y, goal.z * speed_scale));
    }

    return path;
}

geometry_msgs::msg::PoseStamped LocalPlannerCore::make_pose(double x, double y, double speed) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = params_.local_frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = speed;
    pose.pose.orientation.w = 1.0;
    return pose;
}

const PlanDebug &LocalPlannerCore::last_debug() const {
    return last_debug_;
}
