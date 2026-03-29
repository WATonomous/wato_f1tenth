#include "ebreak.hpp"

/*
add the speed topic for testing and calibration, rightnow it is looking for 
speed from odom, but there is not odom
goal tommorw: 
shrink the view cone and play with the ttc timings more, too lose rn tigher
*/

EBREAK_NODE::EBREAK_NODE() : Node ("ebreak_node") {

    //topics
    this->declare_parameter<std::string>("odom_topic","/odom");
    //this->declare_parameter<std::string>("speed_topic","/autodrive/f1tenth_1/speed");
    this->declare_parameter<std::string>("laser_topic","/scan");
    this->declare_parameter<std::string>("ackermann_output_topic","/drive/ebreak");
    this->declare_parameter<std::string>("ackermann_mon_topic","ackermann_cmd");
    this->declare_parameter<std::string>("ackermann_steering_topic","/drive/joystick");

    //ttc time stages (lower the time, the more breaking need to be applied)
    this->declare_parameter<double>("ttc1", 0.39);
    this->declare_parameter<double>("ttc2", 0.23);
    this->declare_parameter<double>("ttc3", 0.15);

    //ttc reduction factor stage 1 = 40% reduction, stage 2 = 70% reduction, stage 3 = 100% reduction
    //must me tweaked and adjusted
    this->declare_parameter<double>("ttc_rf_1",0.9);
    this->declare_parameter<double>("ttc_rf_2", 1.0);
    this->declare_parameter<double>("ttc_rf_3", 1.0);
    
    // at what speed the emergancy break won't activate, to save compute
    this->declare_parameter<double>("speed_threshold",0.03); 
    this->declare_parameter<double>("look_ofset", 0.2618); // pi/3

    //how many ttc warning need to go off before we activate the emergancy break
    this->declare_parameter<int>("alarm_threshold",2);

    ttc1 = this->get_parameter("ttc1").as_double();
    ttc2 = this->get_parameter("ttc2").as_double();
    ttc3 = this->get_parameter("ttc3").as_double();

    ttc_throtel_1 = this->get_parameter("ttc_rf_1").as_double();
    ttc_throtel_2 = this->get_parameter("ttc_rf_2").as_double();
    ttc_throtel_3 = this->get_parameter("ttc_rf_3").as_double();

    speed_threshold = this->get_parameter("speed_threshold").as_double();
    alarm_threshold = this->get_parameter("alarm_threshold").as_int();
    look_ofset = this->get_parameter("look_ofset").as_double();

    odom_topic = this->get_parameter("odom_topic").as_string();
    laser_topic = this->get_parameter("laser_topic").as_string();
    ackermann_output_topic = this->get_parameter("ackermann_output_topic").as_string();
    ackermann_mon_topic = this->get_parameter("ackermann_mon_topic").as_string();
    steering_topic = this->get_parameter("ackermann_steering_topic").as_string();

    //pubs and subs
    ackermann_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_output_topic, 10);
    
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,10,
        [this] (nav_msgs::msg::Odometry::SharedPtr msg) {current_speed = msg->twist.twist.linear.x;});
    
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topic, 10 , 
        std::bind(&EBREAK_NODE::laser_callback, this, std::placeholders::_1));
    
    //speed_sub = this->create_subscription<std_msgs::msg::Float32>("/autodrive/f1tenth_1/speed",10,
        //[this](std_msgs::msg::Float32::SharedPtr msg) {current_speed = msg->data;});

    ackermann_mon = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(ackermann_mon_topic, 10,
        [this] (ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {current_ackermann_msg = *msg;});

    ackermann_steering_mon = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(steering_topic, 10,
        [this] (ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {current_steering = *msg;});

}

void EBREAK_NODE::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    //stop doing anyting if the speed is too low
    if (current_speed <= speed_threshold) {
        return;
    }

    //RCLCPP_INFO(this->get_logger(), "current speed %f", current_speed);
    //ttc flags

    if (!is_breaking)
    {
        bool stage_1 = false;
        bool stage_2 = false;
        bool stage_3 = false;
        int trigger_counter = 0;
        size_t min_index = (-(M_PI / 4) - msg->angle_min) / msg->angle_increment;
        size_t max_index = ((M_PI / 4) - msg->angle_min) / msg->angle_increment;

        for (size_t i = min_index; i < max_index; i++)
        {

            if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
            {
                continue;
            }

            float angle = msg->angle_min + i * msg->angle_increment;
            float range_dt = current_speed * std::cos(angle);
            float ttc = (msg->ranges[i] - 0.18) / range_dt;

            if (ttc < ttc3)
            {
                trigger_counter++;
                stage_3 = true;
            }
            else if (ttc < ttc2)
            {
                stage_2 = true;
                trigger_counter++;
            }
            else if (ttc < ttc1)
            {
                stage_1 = true;
                trigger_counter++;
            }
            else
            {
                trigger_counter = 0;
                stage_1 = false;
                stage_2 = false;
                stage_3 = false;
            }

            if (trigger_counter >= alarm_threshold)
            {
                is_breaking = true;
                prock_counter = 5;
                if (stage_3)
                {
                    desired_speed = current_speed * (1 - ttc_throtel_3);
                    RCLCPP_INFO(this->get_logger(), "triggered stage 3");
                }
                else if (stage_2)
                {
                    desired_speed = current_speed * (1 - ttc_throtel_2);
                    RCLCPP_INFO(this->get_logger(), "triggered stage 2");
                }
                else if (stage_1)
                {
                    desired_speed = current_speed * (1 - ttc_throtel_1);
                    RCLCPP_INFO(this->get_logger(), "triggered stage 1");
                }
                break;
            }
        }
    }

    if (is_breaking) {

        RCLCPP_INFO(this->get_logger(), "braking");
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        ackermann_msgs::msg::AckermannDrive drive;

        drive.steering_angle = current_steering.drive.steering_angle;
        drive.steering_angle_velocity = current_steering.drive.steering_angle_velocity;
        drive.speed = desired_speed;

        drive_msg.drive = drive;
        drive_msg.header.stamp = msg->header.stamp;
        drive_msg.header.frame_id = "base_link";

        ackermann_pub->publish(drive_msg);
        prock_counter--;

        if (prock_counter <= 0) {
            is_breaking = false;
        }

    }

}

int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EBREAK_NODE>());
    rclcpp::shutdown();
    return 0;
}

