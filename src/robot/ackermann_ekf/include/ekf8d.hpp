#ifndef EKF8D_NODE_
#define EKF8D_NODE_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//use of use and readibility changes
using vector8d = Eigen::Matrix<double,8,1>;
using matrix8d = Eigen::Matrix<double,8,8>;
using vector7d = Eigen::Matrix<double,8,1>;


class EKF8D_NODE : rclcpp::Node {
public:

    EKF8D_NODE();

private:

    //subs 
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throtel_sub ;     
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub; 
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub ; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub; 
    
    //pubs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub; 

    //functions
    void wheelodomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

    //sensor and controls data storage
    std_msgs::msg::Float32 current_steering;
    std_msgs::msg::Float32 current_throtel;   
    std_msgs::msg::Float32 current_imu_data;

    //time
    double dt;
    
    //constants

};

#endif