#include "safety.hpp"

using namespace std::chrono_literals;

class Safety : public rclcpp::Node
{
public:
  Safety(): Node("safety")
  {
    car_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f1tenth_car", 10);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 5, std::bind(&Safety::odom_callback, this, std::placeholders::_1));
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 5, std::bind(&Safety::lidar_callback, this, std::placeholders::_1));
    // drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

  }

private:

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  // rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  visualization_msgs::msg::Marker car_marker;
  visualization_msgs::msg::Marker lidar_scan_markers;
  
  void odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void car_marker_setup(std::string header, float scale[3], float color[4]);
  void lidar_marker_setup(std::string header, float scale[3], float color[4]);

};


void Safety::lidar_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Ran whenever new topic recieved in lidar
    RCLCPP_INFO(this->get_logger(), "Lidar recieved!");
}

void Safety::odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg){
    // Ran whenever new topic recieved in odom
    RCLCPP_INFO(this->get_logger(), "Odom recieved!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Safety>());
  rclcpp::shutdown();
  return 0;
}