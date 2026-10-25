#ifndef KEYBOARD_CONVERTER
#define KEYBOARD_CONVERTER

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class KeyboardConverter : public rclcpp::Node {
public:

    KeyboardConverter();

private:

    //subs and pubs
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub;

    //helper function
    void converter (geometry_msgs::msg::Twist::SharedPtr twist_msg);

    //parameter values
    double top_speed, max_angle;
    std::string twist_topic, ackermann_topic;

};

#endif