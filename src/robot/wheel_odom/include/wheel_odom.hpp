#ifndef WHEEL_ODOM_HPP_
#define WHEEL_ODOM_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class WheelOdom : public rclcpp::Node {
    public :

    WheelOdom();

    private:

    //subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_encoder_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_encoder_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub;

    //publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    //sub data
    sensor_msgs::msg::JointState::SharedPtr right_encoder_data;
    double right_encoder_curret,right_encoder_last;

    sensor_msgs::msg::JointState::SharedPtr left_encoder_data;
    double left_encoder_current,left_encoder_last;

    std_msgs::msg::Float32::SharedPtr steering_data;

    //tf2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::TransformStamped t;
    nav_msgs::msg::Odometry od;

    //timer
    rclcpp::TimerBase::SharedPtr timer_;

    //functions
    void calculateOdom();
    void broadcastTransform();

    //time delta
    const int MS = 20; // mileconds (update interval)
    const double DT = 0.02; // seconds

    //current state
    double x,y,yaw,lin_velocity;

    //update state ?
    bool should_update = false;

    //constants (still need to be tweaked based on vehicle, all values from sim docs)
    const double WHEELBASE = 0.3240; //meters
    const double TRACK_WIDTH = 0.236 ; //meters
    const double WHEEL_RADIUS = 0.0590; //meters
    const int TICKS_PER_REVELOUTION = 4 * 16;
    const double STEERING_NORMAL = 0.5236; 

    //sensor noise vals
    const int ENCODER_ERROR = 2;

};

#endif  