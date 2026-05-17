#ifndef LOCAL_PLANNER__LOCAL_PLANNER_CORE_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_CORE_HPP_

#include <optional>
#include <random>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "local_planner/collision_checker.hpp"
#include "local_planner/rrt_planner.hpp"
#include "local_planner/types.hpp"

class LocalPlannerCore {
public:
    explicit LocalPlannerCore(const CoreParams &params);

    PlanResult plan(
        const geometry_msgs::msg::Point &goal_local,
        const ScanFrame &scan,
        const std::vector<geometry_msgs::msg::Point> &manual_obstacles = {});

    const PlanDebug &last_debug() const;

private:
    std::vector<geometry_msgs::msg::Point> extract_obstacles(const ScanFrame &scan) const;

    std::optional<LocalPath> build_straight_path(
        const geometry_msgs::msg::Point &goal,
        const std::vector<geometry_msgs::msg::Point> &obstacles) const;

    geometry_msgs::msg::PoseStamped make_pose(double x, double y, double speed) const;

    CoreParams params_;
    CollisionChecker checker_;
    RrtPlanner rrt_;
    std::mt19937 rng_;
    PlanDebug last_debug_;
};

#endif  // LOCAL_PLANNER__LOCAL_PLANNER_CORE_HPP_
