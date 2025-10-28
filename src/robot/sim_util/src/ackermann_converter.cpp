#include "ackermann_converter.hpp"

AckermannConverter::AckermannConverter() : Node ("ackerman_conterter") {

    //parameters

    this->declare_parameter("throtel_topic","/autodrive/f1tenth_1/throttle_command");
    this->declare_parameter("steering_topic","/autodrive/f1tenth_1/steering_command");
    this->declare_parameter("input_topic","/ackermann_cmd");

    this->declare_parameter("max_speed",22.88);
    this->declare_parameter("steering_gain",0.5236);

    throtel_topic = this->get_parameter("throtel_topic").as_string();
    steering_topic = this->get_parameter("steering_topic").as_string();
    input_topic = this->get_parameter("input_topic").as_string();

    max_speed = this->get_parameter("max_speed").as_double();
    steering_gain = this->get_parameter("steering_gain").as_double();

    //pubs and subs

    steering_pub = this->create_publisher<std_msgs::msg::Float32>(steering_topic, 10);
    throtel_pub = this->create_publisher<std_msgs::msg::Float32>(throtel_topic, 10);

    ackerman_input_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>
        (input_topic, 10, std::bind(&AckermannConverter::ackermannCallback, this, std::placeholders::_1));

}

void AckermannConverter::ackermannCallback (const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    
    std_msgs::msg::Float32 current_steering, current_throtel;

    current_throtel.data = msg->drive.speed / max_speed; 
    current_steering.data = msg->drive.steering_angle / steering_gain;

    throtel_pub->publish(current_throtel);
    steering_pub->publish(current_steering);

}

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannConverter>());
  rclcpp::shutdown();
  return 0;

}