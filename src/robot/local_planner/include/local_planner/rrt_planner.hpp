#ifndef LOCAL_PLANNER__RRT_PLANNER_HPP_
#define LOCAL_PLANNER__RRT_PLANNER_HPP_

#include <optional>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "local_planner/collision_checker.hpp"
#include "local_planner/types.hpp"

class RrtPlanner {
public:
    RrtPlanner(const CoreParams &params, const CollisionChecker &checker);

    std::optional<LocalPath> plan(
        const geometry_msgs::msg::Point &goal,
        const std::vector<geometry_msgs::msg::Point> &obstacles,
        std::mt19937 &rng);

    const RrtDebug &last_debug() const;

private:
    using PointList = std::vector<geometry_msgs::msg::Point>;

    PointList nodes_to_points(
        const std::vector<RrtNode> &nodes,
        int goal_index,
        const geometry_msgs::msg::Point &goal) const;

    LocalPath densify_path(
        const PointList &path,
        const geometry_msgs::msg::Point &goal) const;

    geometry_msgs::msg::PoseStamped make_pose(double x, double y, double speed) const;

    const CollisionChecker &checker_;
    double planning_distance_;
    double max_lateral_;
    double step_size_;
    double goal_tolerance_;
    int max_iterations_;
    int plan_points_;
    double goal_sample_rate_;
    double path_speed_;
    std::string local_frame_id_;
    RrtDebug last_debug_;
};

#endif  // LOCAL_PLANNER__RRT_PLANNER_HPP_
