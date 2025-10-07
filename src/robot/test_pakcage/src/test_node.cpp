#include "test_node.hpp"


Test_Node::Test_Node() : Node ("test_node") { 
  //publishers
  ekf_theta_pub = this->create_publisher<std_msgs::msg::Float32>("/ekf_theta",10);
  odom_theta_pub = this->create_publisher<std_msgs::msg::Float32>("/odom_theta",10);
  sim_theta_pub = this->create_publisher<std_msgs::msg::Float32>("/sim_theta",10);

  //subscriber
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ekf/odom", 10, 
    std::bind(&Test_Node::odom_callback,this,std::placeholders::_1));

  tf_sub = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, 
    std::bind(&Test_Node::tf_listener,this,std::placeholders::_1));

  wheel_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
    std::bind(&Test_Node::wheel_odom_callback,this,std::placeholders::_1));
  
}

void Test_Node::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  // yaw (Z)
  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  std_msgs::msg::Float32 current_real_yaw;
  current_real_yaw.data = yaw;

  RCLCPP_INFO(this->get_logger(), "ekf odom yaw");
  ekf_theta_pub->publish(current_real_yaw);
}

void Test_Node::tf_listener(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  for (const auto &t : msg->transforms) {
    if (t.header.frame_id == "world" && t.child_frame_id == "f1tenth_1") {

      RCLCPP_INFO(this->get_logger(), "publising tf to odom");
      std_msgs::msg::Float32 current_sim_yaw;

      double x = t.transform.rotation.x;
      double y = t.transform.rotation.y;
      double z = t.transform.rotation.z;
      double w = t.transform.rotation.w;

      // yaw (Z)
      double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

      current_sim_yaw.data = yaw;

      sim_theta_pub->publish(current_sim_yaw);
    }
  }
}

void Test_Node::wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.orientation.x;
  double y = msg->pose.pose.orientation.y;
  double z = msg->pose.pose.orientation.z;
  double w = msg->pose.pose.orientation.w;

  // yaw (Z)
  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  std_msgs::msg::Float32 current_real_yaw;
  current_real_yaw.data = yaw;

  RCLCPP_INFO(this->get_logger(), "wheel odom yaw");
  odom_theta_pub->publish(current_real_yaw);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test_Node>());
  rclcpp::shutdown();
  return 0;
}