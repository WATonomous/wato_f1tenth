#ifndef TRANSFORM_BROADCAST_HPP
#define TRANSFORM_BROADCAST_HPP

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class Transform_Node : public rclcpp::Node {
public:

    Transform_Node();

private:

    //subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    //functions
    void boradCastTransform(const nav_msgs::msg::Odometry::SharedPtr msg);

    //tf2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::TransformStamped t;


};

#endif  