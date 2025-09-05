#include "test_node.hpp"


Test_Node::Test_Node() : Node ("test_node") { 
  //publishers
  real_theta_pub = this->create_publisher<std_msgs::msg::Float32>("/real_theta",10);
  sim_theta_pub = this->create_publisher<std_msgs::msg::Float32>("/sim_theta",10);

  //subscriber
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom/ekf", 10, 
    std::bind(&Test_Node::odom_callback,this,std::placeholders::_1));

  tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, 
    std::bind(&Test_Node::tf_listener,this,std::placeholders::_1));

  
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test_Node>());
  rclcpp::shutdown();
  return 0;
}