#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

class ScanToOccupancyGrid : public rclcpp::Node {
public:
    ScanToOccupancyGrid() : Node("scan_to_occupancy_grid")
    {
        this->declare_parameter<std::string>("grid_topic", "/occupancy_grid");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("base_frame_id", "base_link");
        this->declare_parameter<std::string>("laser_frame_id", "laser");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("width_m", 20.0); //this val is bigger than it need be, realistically 14 works
        this->declare_parameter<double>("height_m", 20.0);//likewise
        this->declare_parameter<int>("occupied_value", 100);
        this->declare_parameter<int>("free_value", 0);
        this->declare_parameter<double>("obstacle_inflation_radius", 0.15);

        grid_topic_ = this->get_parameter("grid_topic").as_string();
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        laser_frame_id_ = this->get_parameter("laser_frame_id").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        width_m_ = this->get_parameter("width_m").as_double();
        height_m_ = this->get_parameter("height_m").as_double();
        occupied_value_ = clampOccupancy(this->get_parameter("occupied_value").as_int());
        free_value_ = clampOccupancy(this->get_parameter("free_value").as_int());
        obstacle_inflation_radius_ = this->get_parameter("obstacle_inflation_radius").as_double();

        
        width_cells_ = std::max(1, static_cast<int>(std::round(width_m_ / resolution_)));
        height_cells_ = std::max(1, static_cast<int>(std::round(height_m_ / resolution_)));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_odom_ = msg;
            });
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                publishGrid(*msg);
            });

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing barebones occupancy grid %dx%d at %.3f m/cell on %s",
            width_cells_, height_cells_, resolution_, grid_topic_.c_str());
    }

private:
    static int8_t clampOccupancy(int value)
    {
        return static_cast<int8_t>(std::clamp(value, 0, 100));
    }

    void publishGrid(const sensor_msgs::msg::LaserScan& scan)
    {
        if (!current_odom_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for odom before publishing occupancy grid");
            return;
        }

        const auto& pose = current_odom_->pose.pose;
        const double ego_x = pose.position.x;
        const double ego_y = pose.position.y;
        const double ego_yaw = tf2::getYaw(pose.orientation);

        const std::string scan_frame_id = scan.header.frame_id.empty() ? laser_frame_id_ : scan.header.frame_id;
        geometry_msgs::msg::TransformStamped base_to_laser;
        try {
            base_to_laser = tf_buffer_->lookupTransform(
                base_frame_id_, scan_frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for TF from %s to %s before publishing occupancy grid: %s",
                base_frame_id_.c_str(), scan_frame_id.c_str(), ex.what());
            return;
        }

        const double laser_x_offset = base_to_laser.transform.translation.x;
        const double laser_y_offset = base_to_laser.transform.translation.y;
        const double laser_yaw_offset = tf2::getYaw(base_to_laser.transform.rotation);
        const double cos_yaw = std::cos(ego_yaw);
        const double sin_yaw = std::sin(ego_yaw);
        const double laser_x = ego_x + cos_yaw * laser_x_offset - sin_yaw * laser_y_offset;
        const double laser_y = ego_y + sin_yaw * laser_x_offset + cos_yaw * laser_y_offset;
        const double laser_yaw = ego_yaw + laser_yaw_offset;

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = scan.header.stamp;
        grid.header.frame_id = frame_id_;
        grid.info.map_load_time = this->now();
        grid.info.resolution = resolution_;
        grid.info.width = static_cast<uint32_t>(width_cells_);
        grid.info.height = static_cast<uint32_t>(height_cells_);
        grid.info.origin.position.x = ego_x - (static_cast<double>(width_cells_) * resolution_) * 0.5;
        grid.info.origin.position.y = ego_y - (static_cast<double>(height_cells_) * resolution_) * 0.5;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(static_cast<size_t>(width_cells_ * height_cells_), free_value_);

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const float range = scan.ranges[i];
            if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
                const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
                const double beam_angle = laser_yaw + angle;
                const double hit_x = laser_x + static_cast<double>(range) * std::cos(beam_angle);
                const double hit_y = laser_y + static_cast<double>(range) * std::sin(beam_angle);

                const int col = static_cast<int>((hit_x - grid.info.origin.position.x) / resolution_);
                const int row = static_cast<int>((hit_y - grid.info.origin.position.y) / resolution_);
                if (col >= 0 && col < width_cells_ && row >= 0 && row < height_cells_) {
                    markInflatedObstacle(grid, col, row);
                }
            }
        }

        grid_pub_->publish(grid);
    }

    void markInflatedObstacle(nav_msgs::msg::OccupancyGrid& grid, int center_col, int center_row)
    {
        const int inflation_cells = static_cast<int>(std::ceil(obstacle_inflation_radius_ / resolution_));
        const double radius_sq = obstacle_inflation_radius_ * obstacle_inflation_radius_;

        for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
            for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
                const double dx = static_cast<double>(dc) * resolution_;
                const double dy = static_cast<double>(dr) * resolution_;
                const int row = center_row + dr;
                const int col = center_col + dc;
                if (dx * dx + dy * dy <= radius_sq &&
                    col >= 0 && col < width_cells_ && row >= 0 && row < height_cells_) {
                    grid.data[static_cast<size_t>(row * width_cells_ + col)] = occupied_value_;
                }
            }
        }
    }

    std::string grid_topic_;
    std::string scan_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    std::string base_frame_id_;
    std::string laser_frame_id_;
    double resolution_;
    double width_m_;
    double height_m_;
    int8_t occupied_value_;
    int8_t free_value_;
    double obstacle_inflation_radius_;
    int width_cells_;
    int height_cells_;

    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToOccupancyGrid>());
    rclcpp::shutdown();
    return 0;
}
