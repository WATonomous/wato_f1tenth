#include "planning_main.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class Planning : public rclcpp::Node
{
public:
  Planning() : Node("minimal_publisher"), CAR_WIDTH(0.25), DIST_PER_STEP(2.0), RES_PER_STEP(25),  VERT_PER_STEP(5), SAMPLE_SIZE(0)
  {
    marker_publisher_  = this->create_publisher<visualization_msgs::msg::Marker>("/vertices_markers", 10);
    drive_publisher_   = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    traj_publisher_    = this->create_publisher<sensor_msgs::msg::PointCloud>("/traj_msg", 10);

    car_subscriber_    = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 5, std::bind(&Planning::car_callback, this, std::placeholders::_1));
    marker_sub_        = this->create_subscription<visualization_msgs::msg::Marker>("/vertices_markers", 10, std::bind(&Planning::marker_callback, this, std::placeholders::_1));

    publisher_timer_   = this->create_wall_timer(1000ms, std::bind(&Planning::publisher_timer_callback, this));

    if(!levine.file_found){
      RCLCPP_INFO(this->get_logger(), "WAS NOT ABLE TO GENERATE MIDLINE");
    }

  }

private:
  // Constants
  const double CAR_WIDTH, DIST_PER_STEP;
  const int RES_PER_STEP, VERT_PER_STEP;
  int SAMPLE_SIZE;
  double MAX_KAPPA;

  // Car data
  Car ego_car;

  // Map data
  std::string package_path = ament_index_cpp::get_package_share_directory("planning");
  std::string centreline_path = package_path + "/assets/centerline.csv";
  std::string raceline_path = package_path + "/assets/raceline.csv";
  Map levine = Map(centreline_path, raceline_path, MAX_KAPPA);

  // Lattice Data
  Lattice lattice = Lattice(levine, DIST_PER_STEP, VERT_PER_STEP, RES_PER_STEP, CAR_WIDTH, SAMPLE_SIZE);

  // Timers
  rclcpp::TimerBase::SharedPtr publisher_timer_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr traj_publisher_;

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

  std::uint8_t getCost(const std::vector<Point>& traj, bool on_raceline);

  void generate_traj();
  void publish_traj();

  size_t point_count;
  void marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    point_count = msg->points.size();
  }

};

void Planning::car_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  ego_car.update_values(msg->pose.pose);
}

void Planning::publisher_timer_callback(){

  // if(point_count == 0){
  //   publish_markers(); // Publish markers after generating points
  // }
  generate_traj();
  publish_markers();
}

std::uint8_t Planning::getCost(const std::vector<Point>& traj, bool on_raceline){
/*
  Designed around ros2 costmap_2d. Value from costmap is resized and then addtional heuristics
  are added. Value is clamped between 0-255. heuristics account for 

  returns value between 0-255: 

  0 - 152 -> no colision
  153 - 250 -> possibly a colision
  251 - 255 -> definitely a colision
  These values are adjusted for the below weights. See inflation chart for ros2 for occupancy weights.
  Calculate value by first adjusting normal values by weight. Then add max heuristic value to each value. 
*/

  // must add up to 1.0
  double w_o = 0.8; //occupancy weight
  double w_k = 0.05; //curvature weight
  double w_r = 0.15; //raceline weight

  std::uint8_t cost = 0;

  // set cost to be value form ros2 costmap when implemented
  double occupancy_value = 0;
  double raceline_value = on_raceline ? 0 : 255; // 0 if on raceline otherwise 255

  double kappa_value = 0;
  for(auto& point : traj) {
    //normalise kappa to add up to a max 225. 
    //Compares point kappa with max kappa from raceline and normalises to between between 0-255. 
    //Then gets divded by number of points so total sum is between 0-255.
    kappa_value += (( std::clamp(fabs(point.kappa), 0.0, MAX_KAPPA) / MAX_KAPPA) * 255) / traj.size(); 
  }

  cost = static_cast<std::uint8_t>( w_o*occupancy_value + w_k*kappa_value + w_r*raceline_value );

  return cost;
}

void Planning::generate_traj(){

  std::vector<std::vector<Point>> trajs;
  Point pose = Point(ego_car.get_x(), ego_car.get_y(), ego_car.get_theta(), 0.0, 0.0, 1.0); //define initial accel here

  //startup init
  if(current_lane_idx == -1){
    int idx = lattice.find_closest_vertices_idx(ego_car);
    lattice.generateCurve(pose, lattice.get_raceline_vertex(idx), RES_PER_STEP, local_traj);
    current_lane_idx = VERT_PER_STEP;
  }

  //check if car is on raceline idx
  trajs = current_lane_idx == VERT_PER_STEP ? trajs = lattice.getTrajectories(ego_car) : lattice.getTrajectories(ego_car, false, current_lane_idx);
  
  if (trajs.size() > 0) {


    std::uint8_t min_cost = 255; //max cost value
    std::vector<Point> min_traj;

    for(size_t i = 0; i < trajs.size(); i++){
      std::vector<Point> traj = trajs.at(i);
      std::uint8_t cost = i == VERT_PER_STEP ? getCost(traj, true) : getCost(traj, false);
      if(cost < min_cost){
        min_traj = traj; 
      }
    }

    for (auto& point : min_traj) {
      local_traj.push_back(point);
    }
        
    publish_traj();
  }

  if(local_traj.size() > RES_PER_STEP*4){
    local_traj.erase(local_traj.begin(), local_traj.begin()+RES_PER_STEP);
  }

  current_lane_idx = VERT_PER_STEP;

}

void Planning::publish_traj(){
  sensor_msgs::msg::PointCloud trajectories;

  trajectories.header.frame_id = "map";
  trajectories.header.stamp = this->get_clock()->now();

  // Resize for all points
  trajectories.points.resize(local_traj.size());

  // Create additional data channels
  sensor_msgs::msg::ChannelFloat32 theta_channel, kappa_channel, vel_channel, accel_channel, time_channel;
  theta_channel.name = "theta";
  kappa_channel.name = "kappa";
  vel_channel.name = "velocity";
  accel_channel.name = "acceleration";
  time_channel.name = "time";

  for(std::size_t i; i < local_traj.size(); i++){
    
    trajectories.points[i].x = static_cast<float>(local_traj[i].x);
    trajectories.points[i].y = static_cast<float>(local_traj[i].y);
    trajectories.points[i].z = 0.0f;

    theta_channel.values.push_back(static_cast<float>(local_traj[i].theta));
    kappa_channel.values.push_back(static_cast<float>(local_traj[i].kappa));
    vel_channel.values.push_back(static_cast<float>(local_traj[i].vel));
    accel_channel.values.push_back(static_cast<float>(local_traj[i].accel));
    time_channel.values.push_back(static_cast<float>(local_traj[i].time));
  }

  trajectories.channels.push_back(theta_channel);
  trajectories.channels.push_back(kappa_channel);
  trajectories.channels.push_back(vel_channel);
  trajectories.channels.push_back(accel_channel);
  trajectories.channels.push_back(time_channel);

  traj_publisher_->publish(trajectories);

}

//visulization
void Planning::publish_markers(){
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
  marker1.points.clear();

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    std::vector<Point> vertices = lattice.get_vertice_set(i);
    for(const auto& vertex : vertices){
      geometry_msgs::msg::Point vertex_point;
      vertex_point.x = vertex.x;
      vertex_point.y = vertex.y;
      vertex_point.z = 0;

      // RCLCPP_INFO(this->get_logger(), "x=%.2f, y=%.2f, z=%.2f", vertex[0], vertex[1], 0.0);

      marker1.points.push_back(vertex_point);
    }

    Point raceline_point = lattice.get_raceline_vertex(i);
    geometry_msgs::msg::Point vertex_point;
    vertex_point.x = raceline_point.x;
    vertex_point.y = raceline_point.y;
    vertex_point.z = 0;

    marker1.points.push_back(vertex_point);
   }
  
  marker_publisher_->publish(marker1);

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

  //   Point raceline = levine.get_raceline(i);
  //   RCLCPP_INFO(this->get_logger(), "kappa: %f", raceline.kappa);

  //   geometry_msgs::msg::Point point;
  //   point.x = raceline.x;
  //   point.y = raceline..y;
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