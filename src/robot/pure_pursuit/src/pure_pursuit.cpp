#include "pure_pursuit.hpp"

#include <cmath>
#include <fstream> //reading waypoint csv file
#include <sstream> //parsing strings (split csv lines)
#include <chrono> //time durations

#include <tf2/LinearMath/Quaternion.h> //convert quaternion orientation into yaw (heading)
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
using namespace std::chrono_literals; // Allows us to write "20ms" instead of std::chrono::milliseconds(20)

//initialize node + name of node
PurePursuitNode::PurePursuitNode(): Node("pure_pursuit"), x_(0.0), y_(0.0), yaw_(0.0), odom_received_(false) {
  //declare parameters with default values (declare_parameter is a ROS2 method that allows you to declare a ROS parameter)
  //DOUBLE CHECK
  RCLCPP_INFO(this->get_logger(), "Declaring lookahead_distance parameter");
  this->declare_parameter<double>("lookahead_distance", 10); 
  this->declare_parameter<double>("wheelbase", 0.33);
  this->declare_parameter<double>("max_steering_angle", 0.1745); //24 degrees in radians
  this->declare_parameter<double>("speed", 0.1);
  this->declare_parameter<std::string>("waypoints", "/assets/autoDriveRaceline_with_vel.csv");

  //get parameters and store them in member variables
  this->get_parameter("lookahead_distance", lookahead_distance_);
  this->get_parameter("wheelbase", wheelbase_);
  this->get_parameter("max_steering_angle", max_steering_angle_);
  this->get_parameter("speed", speed_);
  this->get_parameter("waypoints", waypoint_file_);

  std::ifstream waypoint_stream(waypoint_file_);
  if (!waypoint_stream.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open waypoint file: %s", waypoint_file_.c_str());
      rclcpp::shutdown();  // Stop ROS cleanly
      return;
  }
  RCLCPP_INFO(this->get_logger(), "Waypoint file loaded: %s", waypoint_file_.c_str());

  //whenever a new odometry message is received, call the odomCallback function
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1)
  );

  // Create a publisher for the Ackermann drive commands
  drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

  // Create a timer to call the drive command function periodically
  timer_ = this->create_wall_timer(
    20ms, 
    std::bind(&PurePursuitNode::controlLoop, this)
  );

  loadWaypoints(waypoint_file_);

  RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node had started !!");

}


void PurePursuitNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract the vehicle's x-position from the odometry message.
    x_ = msg->pose.pose.position.x;

    // Extract the vehicle's y-position from the odometry message.
    y_ = msg->pose.pose.position.y;

    // Convert quaternion orientation into yaw angle (heading).
    yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);

    // Set the flag indicating that odometry has been received.
    // This prevents the controller from running before valid state is available.
    odom_received_ = true;
}

// This function runs periodically (every 20 ms ~ set according to the timer_)
// It computes and publishes steering commands.
void PurePursuitNode::controlLoop()
{
    // If we haven't received odometry yet OR no waypoints exist, do nothing.
    if (!odom_received_ || waypoints_.empty())
        return;

    /* 1. Find closest waypoint */
    // Find the index of the waypoint closest to the current vehicle position.
    int closest_idx = findClosestWaypoint();

    /* 2. Find lookahead point */
    // Starting from the closest waypoint, find the first waypoint
    // that is at least lookahead_distance_ away.
    geometry_msgs::msg::Point lookahead =
        findLookaheadPoint(closest_idx);

    /* 3. Compute curvature */
    // Compute curvature needed to reach the lookahead point.
    double curvature = computeCurvature(lookahead);

    /* 4. Compute steering angle */
    // Convert curvature into a steering angle using the vehicle model.
    double steering_angle = computeSteering(curvature);

    /* 5. Publish drive command */
    // Create a drive message to send to the vehicle.
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;

    drive_msg.header.stamp = now();
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed_;
    
    // Publish the command to the /drive topic.
    drive_publisher_->publish(drive_msg);
}


// Loads waypoints from a CSV file into the waypoints_ vector.
void PurePursuitNode::loadWaypoints(const std::string & file_path)
{
    // If no file path was provided, warn and exit.
    if (file_path.empty()) {
        RCLCPP_WARN(get_logger(), "No waypoint file provided");
        return;
    }

    // Attempt to open the file.
    std::ifstream file(file_path);

    // If the file cannot be opened, log an error and exit.
    if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open waypoint file: %s", file_path.c_str());
        return;
    }

    // Read the file line by line.
    std::string line;
    while (std::getline(file, line)) {

        // Skip empty lines.
        if (line.empty()) continue;

        // Create a string stream from the line.
        std::stringstream ss(line);

        // Temporary strings for x, y, and optional speed values.
        std::string x_str, y_str, speed_str;

        // Read x value (up to comma).
        if (!std::getline(ss, x_str, ',')) continue;

        // Read y value (up to comma).
        if (!std::getline(ss, y_str, ',')) continue;

        // Optional: read speed if present.
        std::getline(ss, speed_str, ',');

        // Convert strings to doubles safely.
        double x = 0.0, y = 0.0, speed = 0.0;
        try {
            x = std::stod(x_str);
            y = std::stod(y_str);
            if (!speed_str.empty()) speed = std::stod(speed_str);
        } catch (const std::invalid_argument &e) {
            RCLCPP_WARN(get_logger(), "Skipping invalid line: %s", line.c_str());
            continue;
        }

        // Create a geometry_msgs::Point to store the waypoint.
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;  // z is unused for planar driving

        // Add waypoint to the vector.
        waypoints_.push_back(p);
    }

    // Log how many waypoints were loaded.
    RCLCPP_INFO(
        get_logger(),
        "Loaded %zu waypoints", waypoints_.size());
}


// Finds the waypoint closest to the current vehicle position.
int PurePursuitNode::findClosestWaypoint() const
{
    // Initialize minimum distance to a very large number.
    double min_dist = std::numeric_limits<double>::infinity();

    // Index of the closest waypoint found so far.
    int best_idx = 0;

    // Current vehicle position stored as a Point.
    geometry_msgs::msg::Point current;
    current.x = x_;
    current.y = y_;

    // Loop over all waypoints.
    for (size_t i = 0; i < waypoints_.size(); ++i) {

        // Compute distance from vehicle to waypoint.
        double d = distance(current, waypoints_[i]);

        // Update closest waypoint if this one is nearer.
        if (d < min_dist) {
            min_dist = d;
            best_idx = static_cast<int>(i);
        }
    }

    // Return index of closest waypoint.
    return best_idx;
}


// Finds a waypoint ahead of the vehicle at least lookahead_distance_ away.
geometry_msgs::msg::Point
geometry_msgs::msg::Point PurePursuitNode::findLookaheadPoint(int start_index) const
{
    // Current vehicle position.
    geometry_msgs::msg::Point current;
    current.x = x_;
    current.y = y_;

    // Search forward along the path starting from the closest waypoint.
    for (size_t i = start_index; i < waypoints_.size(); ++i) {

        // If this waypoint is far enough ahead, use it.
        if (distance(current, waypoints_[i]) >= lookahead_distance_) {
            return waypoints_[i];
        }
    }

    // If no waypoint is far enough (end of path):
    // Return the last waypoint instead of wrapping around.
    return waypoints_.back();
}

// Computes curvature required to reach the lookahead point.
double PurePursuitNode::computeCurvature(
    const geometry_msgs::msg::Point & lookahead) const
{
    // Compute difference between lookahead point and vehicle position.
    double dx = lookahead.x - x_;
    double dy = lookahead.y - y_;

    // Transform point into the vehicle's local coordinate frame.
    double x_local =  std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    double y_local = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;

    // Pure Pursuit curvature formula.
    return (2.0 * y_local) /
           (lookahead_distance_ * lookahead_distance_);
}

// Converts curvature into a steering angle.
double PurePursuitNode::computeSteering(double curvature) const
{
    // Compute steering angle using bicycle model.
    double steering = std::atan(wheelbase_ * curvature);

    // Clamp steering to maximum limits.
    if (steering > max_steering_angle_)
        steering = max_steering_angle_;
    else if (steering < -max_steering_angle_)
        steering = -max_steering_angle_;

    // Return final steering angle.
    return steering;
}

// Computes Euclidean distance between two 2D points.
double PurePursuitNode::distance(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b) const
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Extracts yaw (heading) from a quaternion.
double PurePursuitNode::getYawFromQuaternion(
    const geometry_msgs::msg::Quaternion & q) const
{
    // Convert geometry_msgs quaternion to tf2 quaternion.
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);

    // Convert quaternion to rotation matrix.
    tf2::Matrix3x3 m(quat);

    // Roll, pitch, yaw angles.
    double roll, pitch, yaw;

    // Extract roll, pitch, yaw.
    m.getRPY(roll, pitch, yaw);

    // Return yaw only.
    return yaw;
}

double PurePursuitNode::f(double x) {
  return -.25*sin(x);
  //return -.3*atan(x);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}