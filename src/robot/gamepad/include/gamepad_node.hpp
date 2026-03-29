#ifndef GAMEPAD_NODE
#define GAMEPAD_NODE

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class GamePad : public rclcpp::Node {
public:

    GamePad();

private:

    //pubs and subs
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throtel_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackerman_pub;


    //functions
    void gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    float trigger_maper(const float r2);

    //constants 
    float MAX_SPEED = 22.88;

};

#endif