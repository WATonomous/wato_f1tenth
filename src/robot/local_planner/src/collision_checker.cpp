#include "local_planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
double distance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}
}  // namespace

CollisionChecker::CollisionChecker(
    double planning_distance,
    double max_lateral,
    double obstacle_inflation)
    : planning_distance_(planning_distance),
      max_lateral_(max_lateral),
      obstacle_inflation_(obstacle_inflation) {}

bool CollisionChecker::point_is_free(
    double x,
    double y,
    const std::vector<geometry_msgs::msg::Point> &obstacles) const {
    return point_clearance(x, y, obstacles) >= 0.0;
}

double CollisionChecker::point_clearance(
    double x,
    double y,
    const std::vector<geometry_msgs::msg::Point> &obstacles) const {
    if (x < 0.0 || x > planning_distance_ || std::abs(y) > max_lateral_) {
        return -obstacle_inflation_;
    }

    double min_distance = std::numeric_limits<double>::infinity();
    for (const auto &obstacle : obstacles) {
        min_distance = std::min(min_distance, distance(x, y, obstacle.x, obstacle.y));
    }

    if (!std::isfinite(min_distance)) {
        return planning_distance_;
    }

    return min_distance - obstacle_inflation_;
}

bool CollisionChecker::segment_is_free(
    double x1,
    double y1,
    double x2,
    double y2,
    const std::vector<geometry_msgs::msg::Point> &obstacles) const {
    return segment_clearance(x1, y1, x2, y2, obstacles) >= 0.0;
}

double CollisionChecker::segment_clearance(
    double x1,
    double y1,
    double x2,
    double y2,
    const std::vector<geometry_msgs::msg::Point> &obstacles) const {
    const double length = distance(x1, y1, x2, y2);
    const int checks = std::max(1, static_cast<int>(std::ceil(length / (obstacle_inflation_ * 0.5))));

    double min_clearance = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= checks; ++i) {
        const double t = static_cast<double>(i) / checks;
        const double x = x1 + t * (x2 - x1);
        const double y = y1 + t * (y2 - y1);
        min_clearance = std::min(min_clearance, point_clearance(x, y, obstacles));
    }

    return min_clearance;
}

double CollisionChecker::obstacle_inflation() const {
    return obstacle_inflation_;
}
