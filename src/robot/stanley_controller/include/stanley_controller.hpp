/*
    Authors           : Jun Kim
    Last Date of Edit : 2026-06-15

    Stanley Controller Node

    Implementation overview:
    - A 50 ms timer drives control_timer_callback which updates the state
      machine (INACTIVE / GLOBAL_FOLLOW) and applies the Stanley control law.
    - The front axle position is computed from odometry using the wheelbase
      parameter, as Stanley measures errors at the front axle, not the center of the vehicle.
    - Cross track error is the perpendicular distance from the front axle to
      the closest point on the global path, measured in the car frame (y value).
    - Heading error is the difference between the car's current heading and
      the path heading at the closest waypoint.
    - Stanley formula: steering = (k_h * heading_error) + atan2(k_e * cte, speed)
    - Waypoints are encoded as geometry_msgs::Point where (x, y) is the 2D
      position and z carries the target velocity at that point.

    Testing and verification:
    - Successfully completed several laps on the track autonomously
    - Oscillation on straights observed and tunable via k_e parameter
    - No major difference in laptime compared to pure pursuit at current speed limits




    - autosim trouble if not in focus

*/

#ifndef STANLEY_CONTROLLER_HPP_ 
#define STANLEY_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "visualization_msgs/msg/marker_array.hpp"


enum stanley_state_ {
    INACTIVE,
    GLOBAL_FOLLOW
};

class Stanley_Controller_Node : public rclcpp::Node {
public:
    Stanley_Controller_Node();

private:
    //publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controls_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cte_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_err_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_term_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cte_term_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr delta_pub_;

    //subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dead_man_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  

    //timer
    rclcpp::TimerBase::SharedPtr control_loop_timer;

    //callback functions
    void control_timer_callback();

    //helpers
    void init_parameters();
    void update_controller_state();
    ackermann_msgs::msg::AckermannDriveStamped calculate_control();
    ackermann_msgs::msg::AckermannDriveStamped dead_stop();

    size_t find_closest_point(double x, double y);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    void publish_debug_vis(const geometry_msgs::msg::Pose& base_link_pose,
                       size_t closest_idx,
                       double cross_track_error,
                       double heading_error,
                       double heading_term,
                       double cte_term,
                       double steering_cmd,
                       double velocity);

    //parameters
    std::string global_path_topic;  
    std::string dead_man_active_topic;
    std::string ackermann_control_topic;
    std::string odom_topic;
    std::string speed_topic;

    std::string global_frame_id;
    std::string local_frame_id;

    bool speed_limit_enable;
    double speed_limit;
    double max_steering_angle;
    double k_e;        // cross track error gain
    double k_h;        // heading error gain
    double wheelbase;  // distance between front and rear axles
    double current_velocity;
    bool enable_debug_vis;

    size_t last_closest_idx_;
    double last_cross_track_error_;
    double last_heading_error_;
    double last_heading_term_;
    double last_cte_term_;
    double last_steering_cmd_;
    


    //internal state and variables
    stanley_state_ controller_state;
    std_msgs::msg::Bool dead_man_active;
    nav_msgs::msg::Path current_global_path;
    nav_msgs::msg::Odometry current_pose;
};

#endif