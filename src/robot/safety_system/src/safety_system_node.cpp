#include "safety_system_node.hpp"


SafetyNode::SafetyNode () : Node("SafetySystem") {
    //initalize variables
    pram.max_angle = 60;
    pram.min_angle = -60;
    pram.TTC_stage1 = 10;
    pram.TTC_stage1 = 30;
    pram.TTC_stage1 = 30;

    //define the subs
    laider_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/autodrive/f1tenth_1/lidar", 10, std::bind(&SafetyNode::laiderCallBack, this, std::placeholders::_1));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "/autodrive/f1tenth_1/imu",10,[this](const sensor_msgs::msg::Imu::SharedPtr msg){imu_data = msg;});

    //define the pub
    vel_pub = this->create_publisher<std_msgs::msg::Float32>("/autodrive/f1tenth_1/throttle",10);

    //define the fist time instance
    last_time = this->now();

    RCLCPP_INFO(this->get_logger(),"initalized saftey constructor");
   
} 

void SafetyNode::laiderCallBack(const sensor_msgs::msg::LaserScan::SharedPtr laider_msg) {
    //edge case:: car velocity is o, then ttc is infinity so rule out that case
    

    //set the max and minx looking view cone
    //index = round((angle_rad - angle_min) / angle_increment)
    int max_index = std::round(((pram.max_angle * M_PI / 180) - laider_msg->angle_min)/laider_msg->angle_increment);
    int min_index = std::round(((pram.min_angle * M_PI / 180) - laider_msg->angle_min)/laider_msg->angle_increment);

}

int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}

