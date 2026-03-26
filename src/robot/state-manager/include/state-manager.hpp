#ifndef STATE_MANAGER_HPP_
#define STATE_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

enum STATE {
    RACING_MODE,
    OVERTAKE_BEHIND,
    OVERTAKE_BESIDE,
    OVERTAKE_AHEAD
};

class StateManager : public rclcpp::Node {
public:

    StateManager();
    ~StateManager();

private:

    //publishers
    rclcpp::Publisher<geometry_msgs::msg::Point> goal_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool> active_overtake_pub_;

    //subs

    rclcpp::Subscription<nav_msgs::msg::Path> global_path_sub_;
    //subjct to change, removing soon as tf2 can do the tranfrom for us
    rclcpp::Subscription<nav_msgs::msg::Odometry> odom_sub_; 
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid> occupancy_grid_sub_;

    //parameters
    std::string goal_point_topic, active_overtake_topic, global_path_topic, odom_topic, occupancy_grid_topic;

    //variables
    int state_;
    nav_msgs::msg::Path global_path;
    double max_sensor_distance;

    //callbacks functions
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg);
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr path_msg);

    //helper function (names change in progress)
    nav_msgs::msg::OccupancyGrid create_sdf (const nav_msgs::msg::OccupancyGrid &msg);
    std::vector<geometry_msgs::msg::Point> local_windows_points ();
    std::optional<geometry_msgs::msg::Point> travel_sdf (const nav_msgs::msg::OccupancyGrid &sdf_grid);

};

#endif