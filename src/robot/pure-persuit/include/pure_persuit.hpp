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

    //tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //callback functions
    void control_timer_callback ();
    //void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //helpers
    void init_parameters();

    void update_controller_state();
    std::optional<geometry_msgs::msg::Point> get_local_waypoint();
    std::optional<geometry_msgs::msg::Point> get_global_waypoint();

    ackermann_msgs::msg::AckermannDriveStamped calculate_control(const geometry_msgs::msg::Point &target_point);
    ackermann_msgs::msg::AckermannDriveStamped dead_stop();

    double find_distance(geometry_msgs::msg::Pose current_location, geometry_msgs::msg::Pose destination);
    size_t find_current_position_index();
    std::optional<geometry_msgs::msg::Point> find_lookahead_global(size_t current_vehicle_index);
    std::optional<geometry_msgs::msg::Point> convert_to_local_frame(const geometry_msgs::msg::Point &global_point);
    geometry_msgs::msg::Point transfrom_point_ (const geometry_msgs::msg::Point &point_, const geometry_msgs::msg::Transform &t_);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
    size_t init_position_index_cache();

    //parameters
    std::string global_frame_id, local_frame_id;
    std::string global_path_topic, local_path_topic;
    std::string overtake_ready_topic, dead_man_active_topic;
    std::string ackermann_control_topic, odom_topic;
    bool overtaking_enable, speed_limit_enable;
    double look_ahead_distance, speed_limit, wheel_base, max_steering_angle;
    double kp_gain;

    //internal state and variabels
    state_ controller_state ;
    std_msgs::msg::Bool dead_man_active, overtake_active;
    nav_msgs::msg::Path current_global_path;
    nav_msgs::msg::Path current_local_path;
    nav_msgs::msg::Odometry current_pose;

};

#endif