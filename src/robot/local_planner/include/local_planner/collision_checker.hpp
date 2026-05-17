#ifndef LOCAL_PLANNER__COLLISION_CHECKER_HPP_
#define LOCAL_PLANNER__COLLISION_CHECKER_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"

class CollisionChecker {
public:
    CollisionChecker(double planning_distance, double max_lateral, double obstacle_inflation);

    bool point_is_free(
        double x,
        double y,
        const std::vector<geometry_msgs::msg::Point> &obstacles) const;

    bool segment_is_free(
        double x1,
        double y1,
        double x2,
        double y2,
        const std::vector<geometry_msgs::msg::Point> &obstacles) const;

    double point_clearance(
        double x,
        double y,
        const std::vector<geometry_msgs::msg::Point> &obstacles) const;

    double segment_clearance(
        double x1,
        double y1,
        double x2,
        double y2,
        const std::vector<geometry_msgs::msg::Point> &obstacles) const;

    double obstacle_inflation() const;

private:
    double planning_distance_;
    double max_lateral_;
    double obstacle_inflation_;
};

#endif  // LOCAL_PLANNER__COLLISION_CHECKER_HPP_
