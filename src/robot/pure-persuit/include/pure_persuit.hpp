/*
    Author : Muhtasim Ahsan
    Last Date of Edit : 2026-04-02
    Description : a standard implemntaion of pure persuit
*/
#ifndef PURE_PERSUIT_NODE_HPP_
#define PURE_PERSUIT_NODE_HPP_


#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

enum state_ {
    INACTIVE,
    GLOBAL_FOLLOW,
    LOCAL_FOLLOW
};

class Pure_Persuit_Node : public rclcpp::Node {
public:
    Pure_Persuit_Node();
private:

    //publishers
    rclcpp::Publisher< ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controls_pub_;

    //subscription
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overtake_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dead_man_sub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    //timer
    rclcpp::TimerBase::SharedPtr control_loop_timer;

    //callback functions
    void control_timer_callback ();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //helpers
    void init_parameters();

    void update_controller_state();
    void get_local_waypoint(geometry_msgs::msg::Point &current_point, double &current_velocity);
    void get_global_waypoint(geometry_msgs::msg::Point &current_point, double &current_velocity);

    ackermann_msgs::msg::AckermannDriveStamped calculate_control(const geometry_msgs::msg::Point &target_point, const double &velocity);
    ackermann_msgs::msg::AckermannDriveStamped dead_stop();

    double find_distance(geometry_msgs::msg::Pose current_location, geometry_msgs::msg::Pose destination);
    int find_current_position_index();
    geometry_msgs::msg::Point convert_to_local_frame();
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    //parameters
    std::string global_frame_id, local_frame_id;
    std::string global_path_topic, local_path_topic;
    std::string overtake_ready_topic, dead_man_active_topic;
    std::string ackermann_control_topic, odom_topic;
    bool overtaking_enable, speed_limit_enable;
    double look_ahead_distance, speed_limit, wheel_base, max_steering_angle;
    double pose_margine;

    //internal state and variabels
    state_ controller_state ;
    std_msgs::msg::Bool dead_man_active, overtake_active;
    nav_msgs::msg::Path current_global_path;
    nav_msgs::msg::Path current_local_path;
    nav_msgs::msg::Odometry current_pose;

};

#endif