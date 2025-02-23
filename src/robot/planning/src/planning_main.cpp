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
    marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>("/vertices_markers", 10, std::bind(&Planning::marker_callback, this, std::placeholders::_1));

    publisher_timer_ = this->create_wall_timer(1000ms, std::bind(&Planning::publisher_timer_callback, this));

    if(!levine.file_found){
      RCLCPP_INFO(this->get_logger(), "WAS NOT ABLE TO GENERATE MIDLINE");
    }

  }

private:
  // Car data
  Car ego_car;

  // Map data
  std::string package_path = ament_index_cpp::get_package_share_directory("planning");
  std::string centreline_path = package_path + "/assets/centerline.csv";
  std::string raceline_path = package_path + "/assets/raceline.csv";
  Map levine = Map(centreline_path,raceline_path);

  // Lattice Data
  int sample_size = 150, vertices_per_step = 5;
  double BUFFER = 0.3;
  Lattice lattice = Lattice(levine, sample_size, vertices_per_step, BUFFER);

  // Timers
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr car_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  
  // Subscriber Callback
  void car_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_markers(); 
  void publisher_timer_callback();

  //trajectory calc
  bool on_raceline = false;
  int current_lane_idx = -1;
  std::vector<Point> local_traj;
  void generate_traj();

  size_t point_count;
  void marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    point_count = msg->points.size();
  }

};

void Planning::car_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ego_car.update_values(msg->pose.pose);
  //RCLCPP_INFO(this->get_logger(), "MSG DATA: Pose: x=%.2f, y=%.2f, z=%.2f, Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //RCLCPP_INFO(this->get_logger(), "Car DATA: Pose: x=%.2f, y=%.2f, z=%.2f, Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f, theta=%.2f", ego_car.get_x(), ego_car.get_y(), ego_car.get_z(), ego_car.get_q().x(), ego_car.get_q().y(), ego_car.get_q().z(), ego_car.get_q().w(), ego_car.get_theta());

}

void Planning::publisher_timer_callback(){

  // if(point_count == 0){
  //   publish_markers(); // Publish markers after generating points
  // }
  generate_traj();
  publish_markers();

  // SAMPLE CODE TO MOVE THE ROBOT STRAIGHT
  // auto straight_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  // // Set driving parameters (slow straight driving)
  // straight_drive_msg.drive.speed = 0.3;  // Slow speed, adjust as needed
  // straight_drive_msg.drive.steering_angle = 1.0; 
  // drive_publisher_->publish(straight_drive_msg);
  // Publish the marker
}

void Planning::generate_traj(){

  std::vector<std::vector<Point>> trajs;
  Point pose = Point(ego_car.get_x(), ego_car.get_y(), ego_car.get_theta(), 0.0);

  //startup init
  if(current_lane_idx == -1){
    int idx = lattice.find_closest_vertices_idx(ego_car);
    lattice.generateCurve(pose, lattice.get_raceline_vertex(idx), 25, local_traj);
    current_lane_idx = vertices_per_step;
  }

  trajs = current_lane_idx == vertices_per_step ? trajs = lattice.getTrajectories(ego_car) : lattice.getTrajectories(ego_car, false, current_lane_idx);

  //replace with cost map
  if (vertices_per_step < trajs.size()) {
    for (auto& point : trajs[vertices_per_step]) {
        local_traj.push_back(point);
    }
  }
  current_lane_idx = vertices_per_step;

}

//visulization
void Planning::publish_markers()
{
  // visualization_msgs::msg::Marker marker;
  // marker.header.frame_id = "map";
  // marker.header.stamp = this->get_clock()->now();
  // marker.ns = "centre_line";
  // marker.id = 0;
  // marker.type = visualization_msgs::msg::Marker::POINTS;
  // marker.action = visualization_msgs::msg::Marker::ADD;
  // marker.scale.x = 0.2;  // Point size
  // marker.scale.y = 0.2;
  // marker.color.a = 1.0;
  // marker.color.r = 1.0;  // Red color
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;
  // marker.points.clear();

  // for(int i=0; i<levine.get_midline_size(); i++){

  //   std::vector<double> midpoint = levine.get_midpoint(i);

  //   geometry_msgs::msg::Point point;
  //   point.x = midpoint.at(0);
  //   point.y = midpoint.at(1);
  //   point.z = 0.0;
  //   marker.points.push_back(point);
  // }
  // marker_publisher_->publish(marker);

  // visualization_msgs::msg::Marker marker1;
  // marker1.header.frame_id = "map";
  // marker1.header.stamp = this->get_clock()->now();
  // marker1.ns = "vertices";
  // marker1.id = 1;
  // marker1.type = visualization_msgs::msg::Marker::POINTS;
  // marker1.action = visualization_msgs::msg::Marker::ADD;
  // marker1.scale.x = 0.3;  // Point size
  // marker1.scale.y = 0.2;
  // marker1.color.a = 1.0;
  // marker1.color.r = 1.0;  // purple
  // marker1.color.g = 0.0;
  // marker1.color.b = 1.0;
  // marker1.points.clear();

  // for (int i = 0; i < sample_size; i++) {
  //   std::vector<Point> vertices = lattice.get_vertice_set(i);
  //   for(const auto& vertex : vertices){
  //     geometry_msgs::msg::Point vertex_point;
  //     vertex_point.x = vertex.x;
  //     vertex_point.y = vertex.y;
  //     vertex_point.z = 0;

  //     // RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f, z=%.2f", vertex[0], vertex[1], 0.0);

  //     marker1.points.push_back(vertex_point);
  //   }

  //   Point raceline_point = lattice.get_raceline_vertex(i);
  //   geometry_msgs::msg::Point vertex_point;
  //   vertex_point.x = raceline_point.x;
  //   vertex_point.y = raceline_point.y;
  //   vertex_point.z = 0;

  //   marker1.points.push_back(vertex_point);
  //  }
  
  // marker_publisher_->publish(marker1);

  // visualization_msgs::msg::Marker marker2;
  // marker2.header.frame_id = "map";
  // marker2.header.stamp = this->get_clock()->now();
  // marker2.ns = "raceline";
  // marker2.id = 2;
  // marker2.type = visualization_msgs::msg::Marker::POINTS;
  // marker2.action = visualization_msgs::msg::Marker::ADD;
  // marker2.scale.x = 0.2;  
  // marker2.scale.y = 0.2;
  // marker2.color.a = 1.0;
  // marker2.color.r = 0.0;  
  // marker2.color.g = 0.6;
  // marker2.color.b = 0.53;
  // marker2.points.clear();

  // for(int i=0; i<levine.get_raceline_size(); i++){

  //   std::vector<double> raceline = levine.get_raceline(i);

  //   geometry_msgs::msg::Point point;
  //   point.x = raceline.at(0);
  //   point.y = raceline.at(1);
  //   point.z = 0.0;
  //   marker2.points.push_back(point);

  // }
  // marker_publisher_->publish(marker2);

  visualization_msgs::msg::Marker marker3;
  marker3.header.frame_id = "map";
  marker3.header.stamp = this->get_clock()->now();
  marker3.ns = "raceline";
  marker3.id = 2;
  marker3.type = visualization_msgs::msg::Marker::POINTS;
  marker3.action = visualization_msgs::msg::Marker::ADD;
  marker3.scale.x = 0.2;  
  marker3.scale.y = 0.2;
  marker3.color.a = 1.0;
  marker3.color.r = 0.0;  
  marker3.color.g = 0.6;
  marker3.color.b = 0.53;
  marker3.points.clear();

  for(const auto& point : local_traj){

    geometry_msgs::msg::Point curve_point;
    curve_point.x = point.x;
    curve_point.y = point.y;
    curve_point.z = 0;

    marker3.points.push_back(curve_point);

  }
  marker_publisher_->publish(marker3);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planning>());
  rclcpp::shutdown();
  return 0;
}