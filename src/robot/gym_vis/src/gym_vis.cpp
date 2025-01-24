#include "gym_vis.hpp"

using namespace std::chrono_literals;

class GymVis : public rclcpp::Node
{
public:
  GymVis(): Node("gym_vis"), count_(0)
  {
    car_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f1tenth_car", 10);
    lidar_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f1tenth_lidar_data", 10);
    //traj_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("f1tenth_traj", 10);

    gym_car_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 5, std::bind(&GymVis::car_callback, this, std::placeholders::_1));
    gym_lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 5, std::bind(&GymVis::lidar_callback, this, std::placeholders::_1));
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    publisher_timer_ = this->create_wall_timer(15ms, std::bind(&GymVis::publisher_timer_callback, this));

    // CAR MARKER SETUP
    float car_scale[3] = {0.60, 0.25, 0.125};
    float car_color[4] = {0.0, 0.0, 1.0, 1.0};
    car_marker_setup("map", car_scale, car_color);

    //LIDAR SCAN SETUP
    float lidar_scale[3] = {0.1, 0.1, 0.1};
    float lidar_color[4] = {0.21, 0.9, 0.0, 1.0};
    lidar_marker_setup("map", lidar_scale, lidar_color);
  }

private:
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lidar_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traj_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gym_car_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr gym_lidar_subscriber_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  visualization_msgs::msg::Marker car_marker;
  visualization_msgs::msg::Marker lidar_scan_markers;
  
  void car_callback (const nav_msgs::msg::Odometry::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publisher_timer_callback();

  void car_marker_setup(std::string header, float scale[3], float color[4]);
  void lidar_marker_setup(std::string header, float scale[3], float color[4]);

  size_t count_;

};

void GymVis::publisher_timer_callback(){

  car_publisher_->publish(car_marker);   
  lidar_publisher_->publish(lidar_scan_markers);

  // SAMPLE CODE TO MOVE THE ROBOT STRAIGHT
  // auto straight_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  // // Set driving parameters (slow straight driving)
  // straight_drive_msg.drive.speed = 0.3;  // Slow speed, adjust as needed
  // straight_drive_msg.drive.steering_angle = 0.0; 

  // drive_publisher_->publish(straight_drive_msg);
  // Publish the marker

}

void GymVis::car_callback (const nav_msgs::msg::Odometry::SharedPtr msg){
    //Set the position and orientation
    car_marker.pose.position.x = msg->pose.pose.position.x;
    car_marker.pose.position.y = msg->pose.pose.position.y;
    car_marker.pose.position.z = msg->pose.pose.position.z + car_marker.scale.z/2;

    car_marker.pose.orientation.x = msg->pose.pose.orientation.x;
    car_marker.pose.orientation.y = msg->pose.pose.orientation.y;
    car_marker.pose.orientation.z = msg->pose.pose.orientation.z;
    car_marker.pose.orientation.w = msg->pose.pose.orientation.w;

    //DEBUG MESSAGES
    // RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f] Orientation: [%.2f, %.2f, %.2f, %.2f]",
    //     msg->pose.pose.position.x,
    //     msg->pose.pose.position.y,
    //     msg->pose.pose.position.z,
    //     msg->pose.pose.orientation.x,
    //     msg->pose.pose.orientation.y,
    //     msg->pose.pose.orientation.z,
    //     msg->pose.pose.orientation.w
    //     );

}

void GymVis::lidar_callback (const sensor_msgs::msg::LaserScan::SharedPtr msg){
    static tf2_ros::Buffer tf_buffer(this->get_clock());
    static tf2_ros::TransformListener tf_listener(tf_buffer);

     // Attempt to lookup the transform from the LiDAR frame to the map frame
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
        transform_stamped = tf_buffer.lookupTransform(
            "map",  // Target frame
            msg->header.frame_id,  // Source frame (e.g., lidar_frame)
            tf2::TimePointZero,  // Use the latest available transform
            100ms  // Timeout
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return;
    }

    // Iterate through the LaserScan ranges and convert them to points
    size_t num_points = msg->ranges.size();
    
    lidar_scan_markers.points.clear();

    for (size_t i = 0; i < num_points; ++i)
    {
        float range = msg->ranges[i];

        // Check if the range is valid (non-infinite or NaN)
        if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max)
        {
            // Calculate the angle for this point
            float angle = msg->angle_min + i * msg->angle_increment;

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            geometry_msgs::msg::PointStamped point_in_source, point_in_map;
            point_in_source.header = msg->header;  // Use the LaserScan's frame
            point_in_source.point.x = range * std::cos(angle);
            point_in_source.point.y = range * std::sin(angle);
            point_in_source.point.z = 0.0;  // Laser scan usually provides 2D data (in the XY plane)

            try {
                tf2::doTransform(point_in_source, point_in_map, transform_stamped);
                // Add the transformed point to the marker
                lidar_scan_markers.points.push_back(point_in_map.point);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Point transformation failed: %s", ex.what());
            }
            
            // DEBUG MESSAGES
            // RCLCPP_INFO(this->get_logger(), "Point: [%.2f, %.2f, %.2f]",
            //     range * std::cos(angle),
            //     range * std::sin(angle),
            //     0.0
            //     );
        }
    }
}

void GymVis::car_marker_setup(std::string header, float scale[3], float color[4]){
    
    // Set the header
    car_marker.header.frame_id = header;
    // Set the type and action
    car_marker.type = visualization_msgs::msg::Marker::CUBE;
    car_marker.action = visualization_msgs::msg::Marker::ADD;

    //Define the size of the rectangle (e.g., 2 meters long, 1 meter wide, 0.5 meter high)
    car_marker.scale.x = scale[0];  //Length
    car_marker.scale.y = scale[1];  //Width
    car_marker.scale.z = scale[2];  //Height (thickness for visualization)

    //Set color (RGBA)
    car_marker.color.r = color[0];
    car_marker.color.g = color[1];
    car_marker.color.b = color[2]; //Blue rectangle
    car_marker.color.a = color[3]; //Fully opaque
    //car_marker.mesh_resource = "package://gym_vis/assets/f1tenth_car.stl";
}

void GymVis::lidar_marker_setup(std::string header, float scale[3], float color[4]){

    lidar_scan_markers.header.frame_id = header;
    // Set the type and action
    lidar_scan_markers.type = visualization_msgs::msg::Marker::POINTS;
    lidar_scan_markers.action = visualization_msgs::msg::Marker::ADD;

      // Set scale (size of points)
    lidar_scan_markers.scale.x = scale[0];  // Width of points (smaller is more visible)
    lidar_scan_markers.scale.y = scale[1];  // Height of points
    lidar_scan_markers.scale.z = scale[2];  // Depth of points

    // Set color (RGBA)
    lidar_scan_markers.color.r = color[0];  // Red
    lidar_scan_markers.color.g = color[1];  // Green
    lidar_scan_markers.color.b = color[2];  // Blue
    lidar_scan_markers.color.a = color[3];  // Fully opaque
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GymVis>());
  rclcpp::shutdown();
  return 0;
}