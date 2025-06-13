#include "gamepad_node.hpp"

GamePad::GamePad() : Node ("gamepad_node") {

    //initalize the pubs and subs
    gamepad_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",10,std::bind(&GamePad::gamepadCallback,this,std::placeholders::_1));

    throtel_pub = this->create_publisher<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/throttle_command",10);
    
    steering_pub = this->create_publisher<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/steering_command",10);
}

void GamePad::gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    //throtel and steering input and thier default values value
    std_msgs::msg::Float32 throtel_comand;    
    std_msgs::msg::Float32 steering_comand;
    
    throtel_comand.data = 0.001;
    steering_comand.data = 0.0;

    //find the current control input
    float current_steering = msg->axes.at(0);
    float current_throtel = trigger_maper(-(msg->axes.at(5)-1)/2);
    float current_break = trigger_maper(-(msg->axes.at(2)-1)/2);
    float total_throtel = 0.0;

    //throtel and breaking logic, 
    if (current_throtel <= 0) current_throtel = 0.0;
    if (current_break <= 0) current_break = 0.0;


    total_throtel = current_throtel - current_break;
    throtel_comand.data = total_throtel;
    steering_comand.data = current_steering;

    throtel_pub->publish(throtel_comand);
    steering_pub->publish(steering_comand);

}

float GamePad::trigger_maper(const float r2) {
    return std::exp(4 * (r2 - 1)) - 0.01832;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamePad>());
  rclcpp::shutdown();
  return 0;
}