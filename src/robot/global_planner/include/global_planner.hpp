#ifndef GLOBAL_PLANNER
#define GLOBAL_PLANNER

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

//message types
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class GlobalPlanner: public rclcpp::Node {
public:
    GlobalPlanner();
private:
    //publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub;

    //helper functions
    void publish_data ();
    void retrive_data (std::ifstream &file);

    //helper variabels
    std_msgs::msg::Float32MultiArray velocities;
    nav_msgs::msg::Path waypoints;
    visualization_msgs::msg::Marker vis_path;

    //parameters
    std::string velocity_pub_topic, path_pub_topic, vis_pub_topic, file_path;
    
};


#endif
