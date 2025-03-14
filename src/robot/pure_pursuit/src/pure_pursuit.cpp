#include <pure_pursuit.hpp>
using namespace std;

class PurePursuit : public rclcpp::Node {

  public:
    PurePursuit(): Node("pure_pursuit") {

      waypoints = {};
      velocities = {};
    
      string package_path = ament_index_cpp::get_package_share_directory("pure_pursuit");
      string file_path = package_path + "/assets/MexicoCity_raceline.csv";

      pointsFile.open(file_path);
      if (pointsFile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Successfully opened waypoints file");
      }
      //addWaypointsRaceline(pointsFile);
      num_waypoints = static_cast<int>(waypoints.size());
      goal_point = {0.17, 0.78};

      // SUBSCRIBERS ------------------------------------------------------------------------------------------------------------------
      odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, bind(&PurePursuit::odomCallback, this, placeholders::_1)
      );

      pose_subscriber_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/f1tenth_car", 10, bind(&PurePursuit::pose_callback, this, placeholders::_1)
      );

      traj_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "/traj_msg", 10, bind(&PurePursuit::traj_callback, this, placeholders::_1)
      );
      // ------------------------------------------------------------------------------------------------------------------------------

      // TIMERS -----------------------------------------------------------------------------------------------------------------------
      odom_timer_ = this->create_wall_timer(
        chrono::seconds(2), 
        bind(&PurePursuit::odomTimerCallback, this));

      drive_timer_ = this->create_wall_timer(
        chrono::milliseconds(100), 
        bind(&PurePursuit::driveTimerCallback, this));

      pose_timer_ = this->create_wall_timer(
        chrono::seconds(2), 
        bind(&PurePursuit::poseTimerCallback, this));
      
      goal_point_task = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PurePursuit::gptask_callback, this)
      );
      // ------------------------------------------------------------------------------------------------------------------------------

      // PUBLISHERS -------------------------------------------------------------------------------------------------------------------
      drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
      // ------------------------------------------------------------------------------------------------------------------------------    
    }

  private:

    vector<pair<double, double>> waypoints;
    vector<double> velocities;
    ifstream pointsFile;
    void addWaypoints(ifstream &file);
    void addWaypointsRaceline(ifstream &file);
    int num_waypoints;

    // Odom Data
    rclcpp::TimerBase::SharedPtr odom_timer_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void odomTimerCallback();

    // Drive
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr drive_timer_;
    double target_speed;
    void driveTimerCallback();

    // Car Pose
    void pose_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void poseTimerCallback();
    rclcpp::TimerBase::SharedPtr pose_timer_;
    visualization_msgs::msg::Marker::SharedPtr current_pose_;
    geometry_msgs::msg::Point carPosition;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr pose_subscriber_;

    // Goal Point
    pair<double, double> getGoalPoint(double ld, geometry_msgs::msg::Point carPos);
    double pointDistance(pair<double, double> car, pair<double, double> point);
    pair<double, double> interpolatedPoint(double ld, pair<double, double> p1, pair<double, double> p2, pair<double, double> car);
    double sgn(double num);
    bool checkSolution(double maxX, double minX, double maxY, double minY, pair<double, double> sol);
    double f(double x);
    pair<double, double> goal_point;
    rclcpp::TimerBase::SharedPtr goal_point_task;
    void gptask_callback();
    int last_point = 0;
    double steering_angle;
    double heading;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr traj_subscriber_;
    void traj_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
    sensor_msgs::msg::PointCloud::SharedPtr current_traj_;
    bool traj_received = false;
};

void PurePursuit::addWaypointsRaceline(ifstream &file) {
  string line;
  bool first_line = true;  


  while (getline(file, line)) {  
    if (first_line) {
      first_line = false;
      continue; 
    }

    stringstream ss(line);
    string s_m_str, x_str, y_str, psi_str, kappa_str, vx_str, ax_str;

    if (!getline(ss, s_m_str, ';') || 
        !getline(ss, x_str, ';') || 
        !getline(ss, y_str, ';') || 
        !getline(ss, psi_str, ';') || 
        !getline(ss, kappa_str, ';') || 
        !getline(ss, vx_str, ';') || 
        !getline(ss, ax_str, ';')) {
      continue; 
    }

    // Convert strings to doubles
    double x = stod(x_str);
    double y = stod(y_str);
    double v = stod(vx_str);  

    waypoints.emplace_back(x, y);
    velocities.emplace_back(v);
  }
}

void PurePursuit::addWaypoints(ifstream &file) {
  string line;
  bool first_line = true;  

  while (getline(file, line)) {  
    if (first_line) {
      first_line = false;
      continue; 
    }

    stringstream ss(line);
    string x_str, y_str;
    
    if (!getline(ss, x_str, ',') || !getline(ss, y_str, ',')) {
      continue;  
    }

    double x = stod(x_str);
    double y = stod(y_str);
    waypoints.emplace_back(x, y);
  }
}



void PurePursuit::traj_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg) {
  current_traj_ = msg;
  int num_points = static_cast<int>(current_traj_->points.size());
  waypoints.clear();
  velocities.clear();
  last_point= 0;

  const sensor_msgs::msg::ChannelFloat32* vel_channel = nullptr;
  for (const auto& channel : current_traj_->channels) {
      if (channel.name == "velocity") {
          vel_channel = &channel;
          break;
      }
  }

  for (int i=0; i < num_points; i++) {
    double x = current_traj_->points[i].x;
    double y = current_traj_->points[i].y;
    double v = static_cast<double>(vel_channel->values[i]);

    waypoints.emplace_back(x, y);
    velocities.emplace_back(v);
    RCLCPP_INFO(this->get_logger(), "VELOCITY: v=%.2f", v);

  }
  num_waypoints = static_cast<int>(waypoints.size());

  RCLCPP_INFO(this->get_logger(), "VELOCITIES: v=%d", static_cast<int>(velocities.size()));

  if(num_points > 0) {
    traj_received = true;
  }
}



// ODOM SUBSCRIBER CALLBACKS -----------------------------------------------------------------------------------------------------------
void PurePursuit::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_odom_msg_ = msg;
  heading = tf2::getYaw(msg->pose.pose.orientation);
}

void PurePursuit::odomTimerCallback() {
  // if (last_odom_msg_ != nullptr) {
  //   RCLCPP_INFO(this->get_logger(), "Odom Data: x=%.2f, y=%.2f", last_odom_msg_->pose.pose.position.x, last_odom_msg_->pose.pose.position.y);
  // } 
  
  // else {
  //   RCLCPP_WARN(this->get_logger(), "No odometry data received yet.");
  // }
}
// ------------------------------------------------------------------------------------------------------------------------------------

// POSE SUBSCRIBER CALLBACKS -----------------------------------------------------------------------------------------------------------
void PurePursuit::pose_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  current_pose_ = msg;
}

void PurePursuit::poseTimerCallback() {
  if (current_pose_ != nullptr) {
    //RCLCPP_INFO(this->get_logger(), "Pose Data: x=%.2f, y=%.2f", current_pose_->pose.position.x, current_pose_->pose.position.y);
    carPosition = current_pose_->pose.position;
    //getGoalPoint(.4, carPosition);
  } 
  
  else {
    RCLCPP_WARN(this->get_logger(), "No pose data received yet.");
  }
}
// ------------------------------------------------------------------------------------------------------------------------------------


// DRIVE PUBLISHER --------------------------------------------------------------------------------------------------------------------
void PurePursuit::driveTimerCallback() {
  auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msg.header.stamp = this->get_clock()->now();

  drive_msg.drive.speed = target_speed;
  drive_msg.drive.steering_angle = steering_angle;
  drive_msg.drive.steering_angle_velocity = 100;

  // Publish the drive command
  drive_publisher_->publish(drive_msg);
}
// ------------------------------------------------------------------------------------------------------------------------------------

// GOAL POINT -------------------------------------------------------------------------------------------------------------------------
void PurePursuit::gptask_callback() {
  if(traj_received) {
    RCLCPP_INFO(this->get_logger(), "VELOCITIES: v=%d", last_point);
    target_speed = 0.15*velocities[last_point]; // meters per second
    double ld = target_speed*2;
    //ld = 0.3;
    goal_point = getGoalPoint(ld, carPosition);

    double carLength = 1;
    double dy = goal_point.second-carPosition.y;
    double dx = goal_point.first-carPosition.x;
    double alpha = atan2(dy, dx);
    double diff = alpha-heading;

    double curvature = 2*sin(diff)/ld;

    steering_angle = atan(curvature*carLength); // radians
  }
}

double PurePursuit::pointDistance(pair<double, double> car, pair<double, double> point) {
  double distance = pow((pow((car.first-point.first), 2)+pow((car.second-point.second), 2)), 0.5);
  return distance;
}

double PurePursuit::sgn(double num) {
  if(num >= 0) {
    return 1;
  }

  else {
    return -1;
  }
}

bool PurePursuit::checkSolution(double maxX, double minX, double maxY, double minY, pair<double, double> sol) {
  if(sol.first >= minX && sol.first <= maxX && sol.second >= minY && sol.second <= maxY) {
    return true;
  }

  else {
    return false;
  }
}

pair<double, double> PurePursuit::interpolatedPoint(double ld, pair<double, double> p1, pair<double, double> p2, pair<double, double> car) {

  // Offset the Circle to the origin
  double x1 = p1.first - car.first;
  double x2 = p2.first - car.first;
  double y1 = p1.second - car.second;
  double y2 = p2.second - car.second;

  double dX = x2 - x1; // delta x
  double dY = y2 - y1; // delta y
  double d = pow((pow(dX, 2) + pow(dY, 2)), 0.5); // distance between points
  double D = (x1*y2) - (x2*y1); // Determinant for matrix
  double discriminant = (pow(ld, 2)*pow(d, 2) - pow(D, 2));


  if(discriminant >= 0) { // We will only care about real solutions

    double sol1_x = ((D*dY + sgn(dY)*dX*pow(discriminant, 0.5)))/pow(d, 2);
    double sol1_y = ((-1*D*dX + abs(dY)*pow(discriminant, 0.5)))/pow(d, 2);

    double sol2_x = ((D*dY - sgn(dY)*dX*pow(discriminant, 0.5)))/pow(d, 2);
    double sol2_y = ((-1*D*dX - abs(dY)*pow(discriminant, 0.5)))/pow(d, 2);

    // Translate back to original position
    pair<double, double> sol1 = {sol1_x + car.first, sol1_y + car.second};
    pair<double, double> sol2 = {sol2_x + car.first, sol2_y + car.second};

    // Check if POIs are within line segment
    double minX = min(p1.first, p2.first);
    double maxX = max(p1.first, p2.first);
    double minY = min(p1.second, p2.second);
    double maxY = max(p1.second, p2.second);

    bool sol1_validity = checkSolution(maxX, minX, maxY, minY, sol1);
    bool sol2_validity = checkSolution(maxX, minX, maxY, minY, sol2);

    if(sol1_validity && sol2_validity) {
      // Find which POI is closer to the second point
      double dP2_sol1 = pow((pow(p2.first-sol1.first, 2) + pow(p2.second-sol1.second, 2)), 0.5);
      double dP2_sol2 = pow((pow(p2.first-sol2.first, 2) + pow(p2.second-sol2.second, 2)), 0.5);

      if(dP2_sol1 < dP2_sol2) {
        return sol1;
      }

      else {
        return sol2;
      }
    }

    else if (sol1_validity) {
      return sol1;
    }

    else if (sol2_validity) {
      return sol2;
    }
  } 

  return {0,0}; // No valid interpolated points were found

}

pair<double, double> PurePursuit::getGoalPoint(double ld, geometry_msgs::msg::Point carPos) {

  pair<double, double> goalPoint = goal_point;
  pair<double, double> carPoint = {carPos.x, carPos.y};

  for(int i=last_point; i < num_waypoints; i++) {
    if(ld - pointDistance(carPoint, waypoints[i]) == 0) { // found point exactly lookahead distance away (MAYBE ADD A BUFFER MARGIN)
      goalPoint = waypoints[i];
      last_point = i;
      //RCLCPP_INFO(this->get_logger(), "Goal Point: x=%.2f y=%.2f", goalPoint.first, goalPoint.second);
      return goalPoint;
    }

    else if(ld - pointDistance(carPoint, waypoints[i]) < 0) { // first point beyond lookahead distance
      int j = i-1;
      while(ld - pointDistance(carPoint, waypoints[j]) < 0 && j > 0) { j--; } // find last point within lookahead distance
      goalPoint = interpolatedPoint(ld, waypoints[i], waypoints[j], carPoint);

      if(goalPoint.first != 0 && goalPoint.second != 0) {
        last_point = j;
        RCLCPP_INFO(this->get_logger(), "Goal Point: x=%.2f y=%.2f   Waypoint: x=%.2f y=%.2f", goalPoint.first, goalPoint.second, waypoints[i].first, waypoints[i].second);
        return goalPoint;
      }
    }
  }
  
  return goalPoint;
}

double PurePursuit::f(double x) {
  return -.25*sin(x);
  //return -.3*atan(x);
}
// ------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
