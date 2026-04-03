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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Pure_Persuit_Node : public rclcpp::Node {
public:
    Pure_Persuit_Node();
private:

    //publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controls_pub_;

    //subscription
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overtake_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dead_man_sub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;

    //timer
    rclcpp::TimerBase::SharedPtr control_loop_timer;

    //callback functions
    void control_timer_callback ();

    //helpers
    void init_parameters();

    //parameters
    std::string global_frame_id, local_frame_id;
    std::string global_path_topic, local_path_topic;
    std::string overtake_ready_topic, dead_man_active_topic;
    std::string ackermann_control_topic;
    bool overtaking_enable;
    double look_ahead_distance;

    //internal state
    bool dead_man_active, overtake_active;
    nav_msgs::msg::Path current_global_path;
    nav_msgs::msg::Path current_local_path;

};

#endif