#include "test_node.hpp"


Test_Node::Test_Node() : Node ("test_node") {

    speed_pub = this->create_publisher<std_msgs::msg::Float32>("/myspeed",10); 
    acc_pub_x = this->create_publisher<std_msgs::msg::Float32>("/adsf",10);
    acc_pub_y = this->create_publisher<std_msgs::msg::Float32>("/adshgh",10);

    throtel_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/autodrive/f1tenth_1/throttle",10,std::bind(&Test_Node::throtelCallback,this,std::placeholders::_1));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/autodrive/f1tenth_1/imu", 10, std::bind(&Test_Node::imu_callback,this,std::placeholders::_1));

    speed = 0;
}

void Test_Node::throtelCallback (const std_msgs::msg::Float32::SharedPtr msg) {

  std_msgs::msg::Float32 current_speed;
  current_speed.data =  22.88 * msg->data;
  //speed_pub->publish(current_speed);

}

void Test_Node::imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg) {

  if (!imu_initalized) {
    prev_imu = *msg;
    imu_initalized = true;
    RCLCPP_INFO(this->get_logger(),"initalized the test imu");
    return;
  }

  double dt = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - (prev_imu.header.stamp.sec + prev_imu.header.stamp.nanosec * 1e-9) ;
  speed += msg->linear_acceleration.x * dt;

  prev_imu = *msg;

  std_msgs::msg::Float32 current_speed;
  current_speed.data = speed;

  speed_pub->publish(current_speed);


}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test_Node>());
  rclcpp::shutdown();
  return 0;
}