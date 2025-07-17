#include "ebreak.hpp"

EBREAK_NODE::EBREAK_NODE() : Node ("ebreak_node") {

    //parameters

    //topics
    this->declare_parameter<std::string>("odom_topic","ekf/odom");
    this->declare_parameter<std::string>("laser_topic","/autodrive/f1tenth_1/lidar");
    this->declare_parameter<std::string>("throtel_command_topic","/autodrive/f1tenth_1/throttle_command");
    this->declare_parameter<std::string>("throtel_topic","/autodrive/f1tenth_1/throttle");

    //ttc parameters
    this->declare_parameter<double>("ttc1", 0.97);
    this->declare_parameter<double>("ttc2", 0.59);
    this->declare_parameter<double>("ttc3", 0.34);
    this->declare_parameter<double>("ttc_throtel_1",0.2);
    this->declare_parameter<double>("ttc_throtel_2", 0.4);
    this->declare_parameter<double>("ttc_throtel_3", 0.4);
    this->declare_parameter<double>("speed_threshold",0.01);
    this->declare_parameter<int>("alarm_threshold",3);

    ttc1 = this->get_parameter("ttc1").as_double();
    ttc2 = this->get_parameter("ttc2").as_double();
    ttc3 = this->get_parameter("ttc3").as_double();

    ttc_throtel_1 = this->get_parameter("ttc_throtel_1").as_double();
    ttc_throtel_2 = this->get_parameter("ttc_throtel_2").as_double();
    ttc_throtel_3 = this->get_parameter("ttc_throtel_3").as_double();

    speed_threshold = this->get_parameter("speed_threshold").as_double();
    alarm_threshold = this->get_parameter("alarm_threshold").as_int();
 
    throtel_command_topic = this->get_parameter("throtel_command_topic").as_string();
    throtel_topic = this->get_parameter("throtel_topic").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    laser_topic = this->get_parameter("laser_topic").as_string();

    //pubs and subs
    throtel_pub = this->create_publisher<std_msgs::msg::Float32>(throtel_command_topic,10);

    throtel_mom_sub = this->create_subscription<std_msgs::msg::Float32>(throtel_topic,10,
        [this] (std_msgs::msg::Float32::SharedPtr msg) {current_throtel = msg->data;});
    
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,10,
        [this] (nav_msgs::msg::Odometry::SharedPtr msg) {current_speed = msg->twist.twist.linear.x;});
    
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topic, 10 , 
        std::bind(&EBREAK_NODE::laser_callback, this, std::placeholders::_1));

}

void EBREAK_NODE::laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {

    float throtel_percent_reduction = 0;
    int tringer_counter = 0;
    bool sending_break = false;
    std_msgs::msg::Float32 throtel_msg;

    // skip if the vehicle velocity is 0, as ttc if than infinity (for now return a 0 cuase there is controls to correct it)
    if (current_speed < speed_threshold) {
        return;
    }

    for (size_t i = 0; i < msg->ranges.size(); i++) {

        // no need to check values outside range
        if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min) {
            continue;
        }

        float angle = msg->angle_min + i * msg->angle_increment;
        float range_dt = current_speed * std::cos(angle);

        // if rate is positive or 0 than the ttc is increase, so safe, or is infinity so also safe
        if (range_dt >= 0) {
            continue;
        }

        // calculate and long ttc
        float ttc = msg->ranges[i] / -range_dt;

        /*insted of a niave approch and relying on one beam,
        its better to see if 3 ttcs are triggered below the thereshorld before
        sending the break signal thats why we get less false positives*/

        // check if the ttc is trigered of discard it as false alarm and reset
        if (ttc < ttc1) {
            
            throtel_percent_reduction += ttc_throtel_1;
            if (ttc < ttc2) {

                throtel_percent_reduction += ttc_throtel_2;

                if (ttc < ttc3) {
                    throtel_percent_reduction += ttc_throtel_3;
                }
            }
            tringer_counter++;

        }
        else {
            tringer_counter = 0;
            throtel_percent_reduction = 0.0;
        }

        if (tringer_counter >= alarm_threshold) {
            sending_break = true;
            break;
        }
    }

    if (sending_break) {

        float throtel_input;

        throtel_input = current_throtel - current_throtel * throtel_percent_reduction;

        if (throtel_input < 0)
            throtel_input = 0;

        else if (throtel_input > 1.0)
            throtel_input = 1.0;

        throtel_msg.data = throtel_input;
        throtel_pub->publish(throtel_msg);
    }
}

int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EBREAK_NODE>());
    rclcpp::shutdown();
    return 0;
}

