#ifndef JOYPAD_NODE
#define JOYPAD_NODE

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

struct button_and_axis {
    int throttle_axis,
        steering_axis, 
        break_axis, 
        safety_button,
        reverse_button,
        pitLimit_button;
};

class JOYPAD : public rclcpp::Node {
public:

    JOYPAD();

private:

    //pubs and subs
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gamepad_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackerman_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dead_man_pub_;

    //functions
    void gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    float trigger_maper(const float r2);
    float steering_mapper(const float ls);

    //parameters
    std::string joy_topic, gamepad_topic, dead_man_topic, my_frame_id;
    float max_speed, max_steering_rate,steering_gain;
    ackermann_msgs::msg::AckermannDrive emergancy_drive_msg;

    //data
    float direction = 1;
    bool reversing = false;

    //buttons
    button_and_axis ba;

};

#endif