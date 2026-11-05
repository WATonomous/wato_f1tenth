/*
    Stanley control law, without ROS plumbing, so unit tests can call it directly.
    The node (stanley_controller.cpp) finds the closest waypoint and calls compute().
*/

#ifndef STANLEY_HPP_
#define STANLEY_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace stanley {

struct Params {
    double k_h;                 // heading error gain
    double k_e;                 // cross track error gain
    double k_soft;              // softening constant, stops atan2 saturating at low speed
    double max_steering_angle;  // rad
    double wheelbase;           // distance between front and rear axles
    bool   speed_limit_active;
    double speed_limit;         // m/s
};

//base_link pose in the path frame, and the measured speed
struct State {
    double x;
    double y;
    double yaw;
    double speed;
};

struct Result {
    double steering;       // rad, clamped
    double speed;          // m/s, from the path, capped
    double cte;            // + = car is right of path, - = left
    double heading_error;  // rad, wrapped to [-pi, pi]
    double heading_term;
    double cte_term;
};

//wrap an angle to [-pi, pi]
double wrap_angle(double a);

//front axle position: stanley measures error here, not at base_link
void front_axle(const State& s, double wheelbase, double& fx, double& fy);

//drop repeated points (closed paths repeat the first pose at the end):
//a zero-length segment gives atan2(0,0) = 0 as the path heading and
//slams the steering to full lock once per lap
void drop_duplicate_points(std::vector<geometry_msgs::msg::PoseStamped>& poses);

//the control law for one tick. closest_idx is the waypoint closest to the
//front axle; path must be non-empty with duplicate points dropped
Result compute(const State& s, const nav_msgs::msg::Path& path, size_t closest_idx, const Params& p);

}  // namespace stanley

#endif
