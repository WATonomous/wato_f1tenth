#ifndef LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "local_planner/local_planner_core.hpp"
#include "local_planner/types.hpp"

struct TrackMapPoint {
    geometry_msgs::msg::Point center;
    geometry_msgs::msg::Point left_boundary;
    geometry_msgs::msg::Point right_boundary;
};

struct ManualObstacle {
    geometry_msgs::msg::Point point_map;
    rclcpp::Time created_at;
};

class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode();

private:
    void init_parameters();
    void plan_timer_callback();
    std::vector<double> build_goal_ladder() const;

    size_t find_nearest_index() const;
    std::optional<geometry_msgs::msg::Point> find_local_goal(
        const geometry_msgs::msg::TransformStamped &transform,
        double lookahead_distance) const;
    geometry_msgs::msg::Point transform_point(
        const geometry_msgs::msg::Point &point,
        const geometry_msgs::msg::TransformStamped &transform) const;
    nav_msgs::msg::Path to_path_msg(const LocalPath &local_path);
    std::vector<geometry_msgs::msg::Point> manual_obstacles_to_local(
        const geometry_msgs::msg::TransformStamped &global_to_local);
    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void clear_manual_obstacles_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void publish_debug_markers(
        const std::string &mode,
        const std::optional<geometry_msgs::msg::Point> &goal_local,
        const std::optional<nav_msgs::msg::Path> &active_path,
        const PlanDebug *debug);
    void publish_track_map();
    void load_track_map();
    void publish_ready(bool ready);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_manual_obstacles_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_active_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_map_pub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::LaserScan scan_;
    bool have_odom_;
    bool have_scan_;

    std::string global_frame_id_;
    std::string global_path_topic_;
    std::string local_path_topic_;
    std::string odom_topic_;
    std::string scan_topic_;
    std::string ready_topic_;
    std::string clicked_point_topic_;
    std::string clear_manual_obstacles_topic_;
    double goal_lookahead_;
    std::vector<double> goal_retry_lookaheads_;
    bool use_scan_obstacles_;
    std::string debug_map_package_;
    std::string debug_map_file_;
    bool debug_map_enabled_;
    bool debug_map_loaded_;
    bool debug_map_published_;
    std::vector<TrackMapPoint> track_map_;
    std::vector<ManualObstacle> manual_obstacles_;
    double manual_obstacle_lifetime_;
    size_t max_manual_obstacles_;

    CoreParams core_params_;
    std::unique_ptr<LocalPlannerCore> core_;
};

#endif  // LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_
