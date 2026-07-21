/*
    Authors           : Jun Kim
    Last Date of Edit : 2026-07-21

    Stanley Controller Node

    
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
    double k_soft;     // softening constant, stops atan2 saturating at low speed
    double wheelbase;  // distance between front and rear axles
    double current_velocity;
    bool enable_debug_vis;

    //internal state and variables
    stanley_state_ controller_state;
    std_msgs::msg::Bool dead_man_active;
    nav_msgs::msg::Path current_global_path;
    nav_msgs::msg::Odometry current_pose;

    //closest-point search state
    size_t prev_closest_idx_ = 0;
    bool   closest_idx_initialized_ = false;
    size_t closest_point_window_ = 20;            // waypoints searched ahead each tick
    double closest_point_recovery_dist_ = 2.0;    // metres; beyond this we re-scan the whole path

    //last computed Stanley terms (for debug viz)
    size_t last_closest_idx_       = 0;
    double last_cross_track_error_ = 0.0;
    double last_heading_error_     = 0.0;
    double last_heading_term_      = 0.0;
    double last_cte_term_          = 0.0;
    double last_steering_cmd_      = 0.0;
};

#endif