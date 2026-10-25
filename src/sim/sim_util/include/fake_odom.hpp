#ifndef FAKE_ODOM
#define FAKE_ODOM

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class FakeOdom : public rclcpp::Node {
public:
    FakeOdom();
private:
    //publisher and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub;

    //tf2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    //helper functions
    void tf_listener (const tf2_msgs::msg::TFMessage::SharedPtr tf_msgs);
    void publish_msgs(const nav_msgs::msg::Odometry &odom_msg);

    //frame names
    std::string child_frame_name;
    std::string header_frame_name;

    //data
    std_msgs::msg::Float32 current_speed;
};

#endif