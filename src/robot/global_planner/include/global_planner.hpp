#ifndef GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

//message types
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/path.hpp"

//#include "visualization_msgs/msg/marker_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class GlobalPlanner: public rclcpp::Node {
public:
    GlobalPlanner();
private:
    //publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    //rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub;

    //helper functions
    void publish_data ();
    void retrieve_data (std::ifstream &file);

    //helper variabels
    nav_msgs::msg::Path waypoints;
    //visualization_msgs::msg::Marker vis_path;

    //parameters
    std::string path_pub_topic, vis_pub_topic, file_directory;
    std::string waypoint_frame_id;
    
};


#endif
