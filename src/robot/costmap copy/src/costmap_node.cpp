#include <chrono>
#include <memory>
#include <string>

#include "costmap_node.hpp"
const int GRIDSIZE = 300;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 50);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidar_sub, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::odom_sub, this, std::placeholders::_1));
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  //timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CostmapNode::publishMessage, this));
}
 
/*
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage(float x) {
  auto message = std_msgs::msg::String();
  message.data = std::to_string(x);
  RCLCPP_INFO(this->get_logger(), "Coord: '%s'", message.data.c_str());
  string_pub_->publish(message);
}*/

// Function to calculate inflated costs
void inflateObstacles(int grid[300][300], double inflationRadius, int maxCost) {
    int inflationCells = static_cast<int>(std::round(inflationRadius * 10));
    for (int x = 0; x < GRIDSIZE; x++) {
        for (int y = 0; y < GRIDSIZE; y++) {
            if (grid[x][y] == 100) {

                for (int dx = -inflationCells; dx <= inflationCells; dx++) {
                    for (int dy = -inflationCells; dy <= inflationCells; dy++) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && nx < GRIDSIZE && ny >= 0 && ny < GRIDSIZE) {
                            double distance = std::sqrt(dx * dx + dy * dy) / 10; 

                            if (distance > inflationRadius) {
                                continue;
                            }

                            int cost = static_cast<int>(maxCost * (1.0 - distance / inflationRadius));

                            grid[nx][ny] = std::max(grid[nx][ny], cost);
                        }
                    }
                }
            }
        }
    }
}

void CostmapNode::publishMessage(int (*grid)[300]) {
    std::stringstream ss;
    ss << "coordinates" << static_cast<int>(std::round(this->x_)) << " " << static_cast<int>(std::round( this->y_))<< " ";
    ss << "\n";
    ss << "array coordinates?" <<this->x_grid_ << " " << this->y_grid_ << " ";
    ss << "\n";
    ss << "direction" <<std::atan2(this->dir_y_, this->dir_x_) <<  "|| ";
    ss << "\n";
    for (int y = 0; y < GRIDSIZE - 1; y+=5) {
      for (int x = 0; x < GRIDSIZE -1 ; x+=5) {
        if(grid[x][y] > 50){
          ss << "X" << " ";
        }
        else{
          ss << ". ";
        }
          
      }
      ss << "\n";
    }
    auto message = std_msgs::msg::String();
    message.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "Costmap:\n%s", message.data.c_str());
    string_pub_->publish(message);
}

void CostmapNode::publishCostmap(int (*arr)[300]){
  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.data.resize(300*300 + 1);
  for (int x = 0; x < 300; ++x){
    for(int y = 0; y < 300; ++y){
      msg.data[x*300 + y] = arr[x][y];
    }
  }
  grid_pub_->publish(msg);
}

void CostmapNode::odom_sub(const nav_msgs::msg::Odometry::SharedPtr odom){
  float float_x = odom->pose.pose.position.x;
  float float_y = odom->pose.pose.position.y;

  double x = odom->pose.pose.orientation.x;
  double y = odom->pose.pose.orientation.y;
  double z = odom->pose.pose.orientation.z;
  double w = odom->pose.pose.orientation.w;

  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  this->x_ = 10*float_x;
  this->y_ = 10*float_y;
  this->dir_x_ = std::cos(yaw);
  this->dir_y_ = std::sin(yaw); 
  this->dir_ = std::atan2(this->dir_y_, this->dir_x_);
  //change direction the robot is facing
  /*
  auto message = std_msgs::msg::String();
  message.data = "X: " + std::to_string(x) + ", Y: " + std::to_string(y);
  // Log and publish the message
  RCLCPP_INFO(this->get_logger(), "Coordinates: '%s'", message.data.c_str());
  string_pub_->publish(message);*/
}

void CostmapNode::lidar_sub(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  auto message = std_msgs::msg::String();
  if(this->x_ == -1 && this->dir_x_ == -100 && this->y_ == -1){
    return;
  }
  float angle = std::atan2(this->dir_y_, this->dir_x_);
  //for(auto i: scan->ranges){
   //publishMessage(i); 
  //}
  int array[300][300] = {0};

  // Step 1: Initialize costmap
    //initializeCostmap();
 
    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double laser_angle = angle + scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            int x_grid = static_cast<int>(range*std::cos(laser_angle)*10 + this->x_) + 150;// add 150 to make bottom corner 0,0
            int y_grid = static_cast<int>(range*std::sin(laser_angle)*10 + this->y_) + 150;// add 150 to make bottom corner 0,0
            this->x_grid_ = x_grid;
            this->y_grid_ = y_grid;
            if(x_grid < 300 && x_grid >= 0 && y_grid < 300 && y_grid >= 0 ){
              array[x_grid][y_grid] = 100;
            }
        }
    }
 
    // Step 3: Inflate obstacles
    inflateObstacles(array, 1.6, 100);
    //publishMessage(array);
    // Step 4: Publish costmap
    auto msg = nav_msgs::msg::OccupancyGrid();
    msg.header = scan->header;
    msg.info.width = 300;
    msg.info.height = 300;
    msg.info.resolution = 0.1;
    msg.info.origin.position.x = -15;  // X coordinate in meters
    msg.info.origin.position.y = -15;  // Y coordinate in meters
//    global_map_.info.origin.position.z = 0.0;  // Z coordinate (usually 0 for 2D maps)

    // Set the orientation as a quaternion (identity orientation)
    msg.info.origin.orientation.x = 0.0;  // Quaternion x
    msg.info.origin.orientation.y = 0.0;  // Quaternion y
    msg.info.origin.orientation.z = 0.0;  // Quaternion z
    msg.info.origin.orientation.w = 1.0; 

    msg.data.resize(300*300);
    for (int x = 0; x < 300; ++x){
      for(int y = 0; y < 300; ++y){
        msg.data[y*300 + x] = array[x][y];
      }
    }
    grid_pub_->publish(msg);
    //publishCostmap(array);
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
