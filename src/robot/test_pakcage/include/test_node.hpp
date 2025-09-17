#ifndef TEST_NODE
#define TEST_NODE

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class Test_Node : public rclcpp::Node {
public : 
    Test_Node();
private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ekf_theta_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr odom_theta_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_theta_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub;

    //function
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void tf_listener(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

};

#endif  