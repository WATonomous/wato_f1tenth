#ifndef PLANNING_STATE_MANAGER_NODE_HPP
#define PLANNING_STATE_MANAGER_NODE_HPP

#include "state_manager_code.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <vector>
#include <string>

namespace local_planning {

class StateManagerNode : public rclcpp::Node {
public:
    StateManagerNode();
    ~StateManagerNode() = default;

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    void stateTimerCallback();
    void planningTimerCallback();

    //utility
    void loadRacingLine();
    void publishState();
    void publishGoal();

    //mainly for visualization in foxglove
    void publishRacingLineLattices(); 

    // conversions
    local_planning::Odometry rosToOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);
    local_planning::OccupancyGrid rosToOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);
    std::vector<local_planning::Point> loadRacingLineFromFile(const std::string& filename);
    geometry_msgs::msg::PointStamped pointToRosPointStamped(const local_planning::Point& point);

    // subs
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // pubs
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_viz_pub_;

    // timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr planning_timer_;

    // state machine 
    std::unique_ptr<RacingStateMachine> state_machine_;

    // cached messages
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_occupancy_grid_;

    // racing line 
    std::vector<local_planning::Point> racing_line_;

    // parameters
    std::string racing_line_file_;
    double state_update_rate_;
    double planning_trigger_rate_;
    double overtake_distance_;
    double lateral_tolerance_;
    double ahead_offset_;
    int num_lattices_;
    double lattice_spacing_;

    // state tracking
    RacingState last_published_state_;
};

} // namespace local_planning

#endif // PLANNING_STATE_MANAGER_NODE_HPP