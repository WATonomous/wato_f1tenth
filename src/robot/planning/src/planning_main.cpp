#include "planning_main.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class Planning : public rclcpp::Node
{
public:
  Planning() : Node("minimal_publisher")
  {
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/vertices_markers", 10);
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    car_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 5, std::bind(&Planning::car_callback, this, std::placeholders::_1));

    publisher_timer_ = this->create_wall_timer(500ms, std::bind(&Planning::publisher_timer_callback, this));

    if(!levine.file_found){
      RCLCPP_INFO(this->get_logger(), "WAS NOT ABLE TO GENERATE MIDLINE");
    }

  }

private:
  // Planner Data
  std::vector<std::vector<double>> midline;
  std::vector<std::vector<std::vector<double>>> midline_vertice_group;
  std::vector<std::vector<double>> raceline_vertices;

  void generate_vertices(int sample_size, int vertices_per_step, double BUFFER);

  // Car data
  Car ego_car;

  // Map data
  std::string package_path = ament_index_cpp::get_package_share_directory("planning");
  std::string centreline_path = package_path + "/assets/centerline.csv";
  std::string raceline_path = package_path + "/assets/raceline.csv";
  Map levine = Map(centreline_path,raceline_path);

  // Timers
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_subscriber_;
  
  // Subscriber Callback
  void car_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_markers(); 
  void publisher_timer_callback();
};

void Planning::car_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ego_car.update_values(msg->pose.pose);
  //RCLCPP_INFO(this->get_logger(), "MSG DATA: Pose: x=%.2f, y=%.2f, z=%.2f, Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //RCLCPP_INFO(this->get_logger(), "Car DATA: Pose: x=%.2f, y=%.2f, z=%.2f, Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f, theta=%.2f", ego_car.get_x(), ego_car.get_y(), ego_car.get_z(), ego_car.get_q().x(), ego_car.get_q().y(), ego_car.get_q().z(), ego_car.get_q().w(), ego_car.get_theta());
 
}

void Planning::publisher_timer_callback(){
  generate_vertices(75, 5, 0.225);
  publish_markers(); // Publish markers after generating points

  // SAMPLE CODE TO MOVE THE ROBOT STRAIGHT
  // auto straight_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  // // Set driving parameters (slow straight driving)
  // straight_drive_msg.drive.speed = 0.3;  // Slow speed, adjust as needed
  // straight_drive_msg.drive.steering_angle = 1.0; 
  // drive_publisher_->publish(straight_drive_msg);
  // Publish the marker
}


void Planning::generate_vertices(int sample_size, int vertices_per_step, double BUFFER)
{
  int step = levine.get_midline_size()/sample_size;
  double VERTEX_X_OFFSET;

  for(int i = 0; i<=levine.get_midline_size() - step; i+=step){

    std::vector<double> base_point = levine.get_midpoint(i);
    std::vector<std::vector<double>> vertices;

    for(int j = 0; j < vertices_per_step; j++){

      if(j <= vertices_per_step/2){
        VERTEX_X_OFFSET = (base_point.at(4) - BUFFER) / (vertices_per_step/2);
      }
      else{
        VERTEX_X_OFFSET = (base_point.at(5) - BUFFER) / (vertices_per_step/2);
      }

      double x = base_point.at(0) + (VERTEX_X_OFFSET * (j - vertices_per_step/2) * std::cos(base_point.at(2) + M_PI_2));
      double y = base_point.at(1) + (VERTEX_X_OFFSET * (j - vertices_per_step/2) * std::sin(base_point.at(2) + M_PI_2));
      double theta = base_point.at(2);
      double kappa = (base_point.at(3) > 1e-6) ? (1 / ((1 / base_point.at(3)) + VERTEX_X_OFFSET)) : 0.0;

      std::vector<double> vertex = {x,y,theta,kappa};
      vertices.push_back(vertex);
    }

    raceline_vertices.push_back(levine.get_closest_raceline(vertices));
    midline_vertice_group.push_back(vertices);
  }
}

//visulization
void Planning::publish_markers()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "centre_line";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.2;  // Point size
  marker.scale.y = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;  // Red color
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.points.clear();

  for(int i=0; i<levine.get_midline_size(); i++){

    std::vector<double> midpoint = levine.get_midpoint(i);

    geometry_msgs::msg::Point point;
    point.x = midpoint.at(0);
    point.y = midpoint.at(1);
    point.z = 0.0;
    marker.points.push_back(point);
  }
  marker_publisher_->publish(marker);

  visualization_msgs::msg::Marker marker1;
  marker1.header.frame_id = "map";
  marker1.header.stamp = this->get_clock()->now();
  marker1.ns = "vertices";
  marker1.id = 1;
  marker1.type = visualization_msgs::msg::Marker::POINTS;
  marker1.action = visualization_msgs::msg::Marker::ADD;
  marker1.scale.x = 0.3;  // Point size
  marker1.scale.y = 0.2;
  marker1.color.a = 1.0;
  marker1.color.r = 1.0;  // purple
  marker1.color.g = 0.0;
  marker1.color.b = 1.0;
  marker1.points.clear();\

  for (const auto& vertices : vertice_group) {
    for(const auto& vertex : vertices){
      geometry_msgs::msg::Point vertex_point;
      vertex_point.x = vertex[0];
      vertex_point.y = vertex[1];
      vertex_point.z = 0;

      // RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f, z=%.2f", vertex[0], vertex[1], 0.0);

      marker1.points.push_back(vertex_point);
    }
   }
  
  marker_publisher_->publish(marker1);

  visualization_msgs::msg::Marker marker2;
  marker2.header.frame_id = "map";
  marker2.header.stamp = this->get_clock()->now();
  marker2.ns = "raceline";
  marker2.id = 2;
  marker2.type = visualization_msgs::msg::Marker::POINTS;
  marker2.action = visualization_msgs::msg::Marker::ADD;
  marker2.scale.x = 0.2;  
  marker2.scale.y = 0.2;
  marker2.color.a = 1.0;
  marker2.color.r = 0.0;  
  marker2.color.g = 0.6;
  marker2.color.b = 0.53;
  marker2.points.clear();

  for(int i=0; i<levine.get_raceline_size(); i++){

    std::vector<double> raceline = levine.get_raceline(i);

    geometry_msgs::msg::Point point;
    point.x = raceline.at(0);
    point.y = raceline.at(1);
    point.z = 0.0;
    marker2.points.push_back(point);

  }
  marker_publisher_->publish(marker2);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planning>());
  rclcpp::shutdown();
  return 0;
}