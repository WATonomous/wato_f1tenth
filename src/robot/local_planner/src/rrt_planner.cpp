#include "local_planner/rrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
double distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}
}  // namespace

RrtPlanner::RrtPlanner(const CoreParams &params, const CollisionChecker &checker)
    : checker_(checker),
      planning_distance_(params.planning_distance),
      max_lateral_(params.max_lateral),
      step_size_(params.step_size),
      goal_tolerance_(params.goal_tolerance),
      max_iterations_(params.max_iterations),
      plan_points_(params.plan_points),
      goal_sample_rate_(params.goal_sample_rate),
      path_speed_(params.path_speed),
      local_frame_id_(params.local_frame_id) {}

std::optional<LocalPath> RrtPlanner::plan(
    const geometry_msgs::msg::Point &goal,
    const std::vector<geometry_msgs::msg::Point> &obstacles,
    std::mt19937 &rng) {
    last_debug_ = RrtDebug{};
    std::uniform_real_distribution<double> x_dist(0.0, planning_distance_);
    std::uniform_real_distribution<double> y_dist(-max_lateral_, max_lateral_);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);

    std::vector<RrtNode> nodes;
    nodes.push_back({0.0, 0.0, -1});
    last_debug_.nodes = nodes;

    for (int j = 0; j < max_iterations_; ++j) {
        const bool sample_goal = unit_dist(rng) < goal_sample_rate_;
        double sample_x = 0.0;
        double sample_y = 0.0;
        if (sample_goal) {
            sample_x = goal.x;
            sample_y = goal.y;
        } else {
            sample_x = x_dist(rng);
            sample_y = y_dist(rng);
        }

        int nearest_idx = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nodes.size(); ++i) {
            const double d = distance(nodes[i].x, nodes[i].y, sample_x, sample_y);
            if (d < nearest_dist) {
                nearest_dist = d;
                nearest_idx = static_cast<int>(i);
            }
        }

        const auto &nearest = nodes[nearest_idx];
        const double theta = std::atan2(sample_y - nearest.y, sample_x - nearest.x);
        const double new_x = nearest.x + step_size_ * std::cos(theta);
        const double new_y = nearest.y + step_size_ * std::sin(theta);

        if (checker_.segment_is_free(nearest.x, nearest.y, new_x, new_y, obstacles)) {
            nodes.push_back({new_x, new_y, nearest_idx});
            last_debug_.nodes = nodes;
            const int new_idx = static_cast<int>(nodes.size() - 1);

            if (distance(new_x, new_y, goal.x, goal.y) <= goal_tolerance_ &&
                checker_.segment_is_free(new_x, new_y, goal.x, goal.y, obstacles)) {
                const auto raw_path = nodes_to_points(nodes, new_idx, goal);
                last_debug_.raw_path = raw_path;
                last_debug_.success = true;
                return densify_path(raw_path, goal);
            }
        }
    }

    return std::nullopt;
}

RrtPlanner::PointList RrtPlanner::nodes_to_points(
    const std::vector<RrtNode> &nodes,
    int goal_index,
    const geometry_msgs::msg::Point &goal) const {
    std::vector<RrtNode> reversed;
    for (int idx = goal_index; idx >= 0;) {
        reversed.push_back(nodes[idx]);
        idx = (nodes[idx].parent < 0) ? -1 : nodes[idx].parent;
    }
    std::reverse(reversed.begin(), reversed.end());

    PointList path;
    path.reserve(reversed.size() + 1);
    for (const auto &node : reversed) {
        geometry_msgs::msg::Point point;
        point.x = node.x;
        point.y = node.y;
        point.z = goal.z;
        path.push_back(point);
    }
    path.push_back(goal);

    return path;
}

LocalPath RrtPlanner::densify_path(
    const PointList &path,
    const geometry_msgs::msg::Point &goal) const {
    LocalPath local_path;
    if (path.empty()) {
        return local_path;
    }

    std::vector<double> accumulated;
    accumulated.reserve(path.size());
    accumulated.push_back(0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        accumulated.push_back(accumulated.back() + distance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y));
    }

    const double total_length = accumulated.back();
    if (total_length <= 1e-9) {
        local_path.poses.push_back(make_pose(path.front().x, path.front().y, goal.z));
        return local_path;
    }

    const int points = std::max(2, plan_points_);
    PointList dense_points;
    dense_points.reserve(points);

    size_t segment = 1;
    for (int i = 0; i < points; ++i) {
        const double target = total_length * static_cast<double>(i) / (points - 1);
        while (segment + 1 < accumulated.size() && accumulated[segment] < target) {
            ++segment;
        }

        const auto &prev = path[segment - 1];
        const auto &next = path[segment];
        const double segment_length = accumulated[segment] - accumulated[segment - 1];
        double t = 0.0;
        if (segment_length > 1e-9) {
            t = (target - accumulated[segment - 1]) / segment_length;
        }

        geometry_msgs::msg::Point point;
        point.x = prev.x + t * (next.x - prev.x);
        point.y = prev.y + t * (next.y - prev.y);
        point.z = goal.z;
        dense_points.push_back(point);
    }

    for (size_t i = 0; i < dense_points.size(); ++i) {
        local_path.poses.push_back(make_pose(
            dense_points[i].x,
            dense_points[i].y,
            goal.z));
    }

    return local_path;
}

geometry_msgs::msg::PoseStamped RrtPlanner::make_pose(double x, double y, double speed) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = local_frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = speed;
    pose.pose.orientation.w = 1.0;
    return pose;
}

const RrtDebug &RrtPlanner::last_debug() const {
    return last_debug_;
}
