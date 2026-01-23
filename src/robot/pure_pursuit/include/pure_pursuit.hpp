#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP
 /*probably dont need this many includes just for function declaration...*/
#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <sstream>

/*ROS Core*/
#include "rclcpp/rclcpp.hpp"

/*ROS Messages*/
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/*TF/Geometry*/
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>

class PurePursuitNode : public rclcpp::Node
{
public:
    PurePursuitNode();
    double f(double x);

private:
    /* ===================== ROS ===================== */

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ================== Parameters ================= */

    double lookahead_distance_;
    double wheelbase_;
    double max_steering_angle_;
    double speed_;
    std::string waypoint_file_;


    /* =================== State ===================== */
    
    double x_;
    double y_;
    double yaw_;
    bool odom_received_;
    std::vector<geometry_msgs::msg::Point> waypoints_;

    /* ============ Callbacks ================== */

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    /* ============ Pure Pursuit Logic ========= */

    int findClosestWaypoint() const;

    geometry_msgs::msg::Point findLookaheadPoint(int start_index) const;

    double computeCurvature(
        const geometry_msgs::msg::Point & lookahead) const;

    double computeSteering(double curvature) const;

    /* ============ Utilities ================== */

    double distance(
        const geometry_msgs::msg::Point & a,
        const geometry_msgs::msg::Point & b) const;

    double getYawFromQuaternion(
        const geometry_msgs::msg::Quaternion & q) const;

    void loadWaypoints(const std::string & file_path);
};

#endif  // PURE_PURSUIT_HPP