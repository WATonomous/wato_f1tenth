#ifndef CONVERTER
#define CONVERTER

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

class AckermannConverter : public rclcpp::Node {
public:
    AckermannConverter();
private:

    //pubs and sub
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackerman_input_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throtel_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;

    //callbacks 
    void ackermannCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    //parameters
    std::string throtel_topic, steering_topic, input_topic;
    double max_speed, steering_gain;
};

#endif
