#include "gym_vis.hpp"

using namespace std::chrono_literals;

class GymVis : public rclcpp::Node
{
public:
  GymVis(): Node("gym_vis"), count_(0)
  {
    car_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f1tenth_car", 10);
    lidar_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f1tenth_lidar_data", 10);
    traj_publisher_ = this->create_publisher<std_msgs::msg::String>("f1tenth_traj", 10);

    gym_car_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&GymVis::car_callback, this, std::placeholders::_1));
    gym_lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&GymVis::lidar_callback, this, std::placeholders::_1));

    // CAR MARKER SETUP

    // Set the header
    car_marker.header.frame_id = "map";
    // Set the type and action
    car_marker.type = visualization_msgs::msg::Marker::CUBE;
    car_marker.action = visualization_msgs::msg::Marker::ADD;

    //Define the size of the rectangle (e.g., 2 meters long, 1 meter wide, 0.5 meter high)
    car_marker.scale.x = 0.60;  //Length
    car_marker.scale.y = 0.25;  //Width
    car_marker.scale.z = 0.125;  //Height (thickness for visualization)

    //Set color (RGBA)
    car_marker.color.r = 0.0;
    car_marker.color.g = 0.0;
    car_marker.color.b = 1.0; //Blue rectangle
    car_marker.color.a = 1.0; //Fully opaque
    //car_marker.mesh_resource = "package://gym_vis/assets/f1tenth_car.stl";

    //LIDAR SCAN SETUP

    lidar_scan_markers.header.frame_id = "map";
    // Set the type and action
    lidar_scan_markers.type = visualization_msgs::msg::Marker::POINTS;
    lidar_scan_markers.action = visualization_msgs::msg::Marker::ADD;

    // Set color (RGBA)
    lidar_scan_markers.color.r = 1.0f;  // Red
    lidar_scan_markers.color.g = 0.0f;  // Green
    lidar_scan_markers.color.b = 0.0f;  // Blue
    lidar_scan_markers.color.a = 1.0f;  // Fully opaque

    // Set scale (size of points)
    lidar_scan_markers.scale.x = 0.1;  // Width of points (smaller is more visible)
    lidar_scan_markers.scale.y = 0.1;  // Height of points
    lidar_scan_markers.scale.z = 0.1;  // Depth of points


  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lidar_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traj_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gym_car_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gym_lidar_subscriber_;

  visualization_msgs::msg::Marker car_marker;
  visualization_msgs::msg::Marker lidar_scan_markers;
  
  void car_callback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  size_t count_;
};

void GymVis::car_callback (const nav_msgs::msg::Odometry::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f] Orientation: [%.2f, %.2f, %.2f, %.2f]",
    //     msg->pose.pose.position.x,
    //     msg->pose.pose.position.y,
    //     msg->pose.pose.position.z,
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w
    //     );

    //Set the position and orientation
    car_marker.pose.position.x = msg->pose.pose.position.x;
    car_marker.pose.position.y = msg->pose.pose.position.y;
    car_marker.pose.position.z = msg->pose.pose.position.z + car_marker.scale.z/2;

    car_marker.pose.orientation.x = msg->pose.pose.orientation.x;
    car_marker.pose.orientation.y = msg->pose.pose.orientation.y;
    car_marker.pose.orientation.z = msg->pose.pose.orientation.z;
    car_marker.pose.orientation.w = msg->pose.pose.orientation.w;

    this->car_publisher_->publish(car_marker);    
}

void GymVis::lidar_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg){

    // Iterate through the LaserScan ranges and convert them to points
    size_t num_points = msg->ranges.size();
    for (size_t i = 0; i < num_points; ++i)
    {
        float range = msg->ranges[i];

        // Check if the range is valid (non-infinite or NaN)
        if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max)
        {
            // Calculate the angle for this point
            float angle = msg->angle_min + i * msg->angle_increment;

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            geometry_msgs::msg::Point point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0;  // Laser scan usually provides 2D data (in the XY plane)

            // Add the point to the marker
            lidar_scan_markers.points.push_back(point);
        }
    }

    // Publish the marker
    lidar_publisher_->publish(lidar_scan_markers);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GymVis>());
  rclcpp::shutdown();
  return 0;
}