#ifndef TEST_NODE
#define TEST_NODE

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Test_Node : public rclcpp::Node {
public : 
    Test_Node();
private:
    //pubs and subs
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr acc_pub_x;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr acc_pub_y;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throtel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;


    //functions
    void throtelCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    //data
    sensor_msgs::msg::Imu prev_imu;
    double speed;
    bool imu_initalized = false;

};

#endif  