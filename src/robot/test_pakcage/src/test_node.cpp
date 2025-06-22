#include "test_node.hpp"


Test_Node::Test_Node() : Node ("test_node") {

    speed_pub = this->create_publisher<std_msgs::msg::Float32>("/myspeed",10); 
    acc_pub_x = this->create_publisher<std_msgs::msg::Float32>("/adsf",10);
    acc_pub_y = this->create_publisher<std_msgs::msg::Float32>("/adshgh",10);

    throtel_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/autodrive/f1tenth_1/throttle",10,std::bind(&Test_Node::throtelCallback,this,std::placeholders::_1));
}

void Test_Node::throtelCallback (const std_msgs::msg::Float32::SharedPtr msg) {

  std_msgs::msg::Float32 current_speed;
  current_speed.data =  22.88 * msg->data;
  speed_pub->publish(current_speed);

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test_Node>());
  rclcpp::shutdown();
  return 0;
}