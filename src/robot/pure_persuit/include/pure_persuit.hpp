/*
    Authors           : Muhtasim Ahsan, Jordan Khatri
    Last Date of Edit : 2026-04-23

    Pure Persuit Node

    This Node is a implemntation of the Pure Persuit algo described in
    the Robo racer docs. This implementation is the general pure persuit
    implementation + the addition of a dynamic look ahead distance to
    improve high speed stability without compromising on need to speed
    accuracy. The node is also driven by a state machine which decides
    wheather to follow the global paht or the local path based off the inputs
    of the semantic planner. The controller will not move the vehicle forward
    unless the deadman switch on the human operated controller is activated.

    Implementation overview :
    - A 50 ms timer drives control_timer_callback which updates the state
      machine (INACTIVE / GLOBAL_FOLLOW / LOCAL_FOLLOW), refreshes the
      lookahead distance from current speed, picks a target waypoint,
      converts it to base_link, and runs the pure pursuit control law.
    - Waypoints are encoded as geometry_msgs::Point where (x, y) is the 2D
      position and z carries the target velocity at that point.
    - Lookahead point selection brackets the waypoint before the lookahead
      circle and the one after it, then calls interpolate_lookahead_point to
      solve |P(t) - ref|^2 = L^2 along that segment. The returned point lies
      exactly on the lookahead circle and its velocity is linearly
      interpolated using the same t parameter. This removes the jitter that
      came from always snapping to a discrete waypoint past the lookahead.

    Testing and verification :
    - the car was able to complete several laps on the track autonomously
    - to verify the dynamic lookahead, a debug topic was made where the
      dynamic look ahead distance was published. this was graphed against
      vehicle speed in foxglove to show the look ahead increase with speed
    - to verify the interpolation, the interpolated lookahead point was
      visualized in Foxglove against the global path: the target point
      slides smoothly along the polyline instead of snapping between
      discrete waypoints, and the published ackermann speed transitions
      smoothly instead of stepping as we move between waypoints with
      different encoded velocities
*/
#ifndef PURE_PERSUIT_NODE_HPP_
#define PURE_PERSUIT_NODE_HPP_


#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

enum state_ {
    INACTIVE,
    GLOBAL_FOLLOW,
    LOCAL_FOLLOW
};

class Pure_Persuit_Node : public rclcpp::Node {
public:
    Pure_Persuit_Node();
private:

    //publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controls_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr look_ahead_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_point_pub_;

    //subscription
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overtake_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dead_man_sub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;

    //timer
    rclcpp::TimerBase::SharedPtr control_loop_timer;

    //tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //callback functions
    void control_timer_callback ();
    //void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //helpers
    void init_parameters();

    void update_controller_state();
    std::optional<geometry_msgs::msg::Point> get_local_waypoint();
    std::optional<geometry_msgs::msg::Point> get_global_waypoint();

    ackermann_msgs::msg::AckermannDriveStamped calculate_control(const geometry_msgs::msg::Point &target_point);
    ackermann_msgs::msg::AckermannDriveStamped dead_stop();

    double find_distance(geometry_msgs::msg::Pose current_location, geometry_msgs::msg::Pose destination);
    size_t find_current_position_index();
    std::optional<geometry_msgs::msg::Point> find_lookahead_global(size_t current_vehicle_index);
    geometry_msgs::msg::Point interpolate_lookahead_point(
        const geometry_msgs::msg::Point &prev_pt,
        const geometry_msgs::msg::Point &curr_pt,
        double ref_x, double ref_y,
        double lookahead);
    std::optional<geometry_msgs::msg::Point> convert_to_local_frame(const geometry_msgs::msg::Point &global_point);
    geometry_msgs::msg::Point transfrom_point_ (const geometry_msgs::msg::Point &point_, const geometry_msgs::msg::Transform &t_);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
    void update_lookahead_distance();
    size_t init_position_index_cache();

    //parameters
    std::string global_frame_id, local_frame_id;
    std::string global_path_topic, local_path_topic;
    std::string overtake_ready_topic, dead_man_active_topic;
    std::string ackermann_control_topic, odom_topic;
    std::string speed_topic;
    bool overtaking_enable, speed_limit_enable;
    double look_ahead_distance, speed_limit, max_steering_angle;
    double kp_gain;
    double max_lookahead, min_lookahead, lookahead_ratio;
    double current_velocity;

    //internal state and variabels
    state_ controller_state ;
    std_msgs::msg::Bool dead_man_active, overtake_active;
    nav_msgs::msg::Path current_global_path;
    nav_msgs::msg::Path current_local_path;
    nav_msgs::msg::Odometry current_pose;

};

#endif