#ifndef JOYPAD_NODE
#define JOYPAD_NODE

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class JOYPAD : public rclcpp::Node {
public:

    JOYPAD();

private:

    //pubs and subs
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackerman_pub;

    //functions
    void gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    float trigger_maper(const float r2);
    float steering_mapper(const float ls);

    //parameters
    std::string joy_topic, gamepad_topic, my_frame_id;
    float max_speed, max_steering_rate,steering_gain;
    ackermann_msgs::msg::AckermannDrive emergancy_drive_msg;
    int steering_axis, throtle_axis, break_axis, dead_man_button, reverse_button, pit_limit_button;

    //data
    float direction = 1;
    bool reversing = false;

};

#endif
