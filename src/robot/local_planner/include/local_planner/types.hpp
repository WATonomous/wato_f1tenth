#ifndef LOCAL_PLANNER__TYPES_HPP_
#define LOCAL_PLANNER__TYPES_HPP_

#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

struct CoreParams {
    bool use_straight_path;
    double planning_distance;
    double path_speed;
    double max_lateral;
    double step_size;
    double goal_tolerance;
    double obstacle_inflation;
    double scan_min_range;
    double scan_max_range;
    double scan_front_min_x;
    double laser_to_base_x;
    double laser_to_base_y;
    int max_iterations;
    int plan_points;
    double goal_sample_rate;
    std::string local_frame_id;
};

struct ScanFrame {
    std::vector<float> ranges;
    double angle_min = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
};

struct LocalPath {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
};

enum class PlanStatus {
    OK,
    NO_PATH_FOUND
};

struct PlanResult {
    PlanStatus status;
    std::optional<LocalPath> path;
};

struct RrtNode {
    double x;
    double y;
    int parent;
};

struct RrtDebug {
    std::vector<RrtNode> nodes;
    std::vector<geometry_msgs::msg::Point> raw_path;
    bool success = false;
};

struct PlanDebug {
    std::vector<geometry_msgs::msg::Point> obstacles;
    RrtDebug rrt;
    bool used_straight = false;
    bool used_rrt = false;
};

#endif  // LOCAL_PLANNER__TYPES_HPP_
