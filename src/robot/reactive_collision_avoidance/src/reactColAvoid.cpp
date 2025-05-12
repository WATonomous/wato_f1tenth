#include "reactColAvoid.hpp"  // Include the corresponding header

// Constructor definition
reactiveColAvoid::reactiveColAvoid()
: Node("reactive_collision_avoidance_node")  // Initialize base Node with name
{
  // Subscriber to LiDAR scan
  scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,                                                         // Topic name and queue size
    std::bind(&reactiveColAvoid::scan_callback, this, std::placeholders::_1) // Bind member callback
  );

  // Subscriber to projected trajectory
  trajectories = this->create_subscription<sensor_msgs::msg::trajectories>(
    "trajectories", 10,
    std::bind(&reactiveColAvoid::traj_callback, this, std::placeholders::_1)
  );

  // Publisher for drive commands
  drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "drive", 10  // Topic name and queue size
  );

  RCLCPP_INFO(this->get_logger(), "reactiveColAvoid node initialized"); // Log initialization
}

// LiDAR scan callback definition
void reactiveColAvoid::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::vector<std::pair<double, double>> gaps;  // Temporary container for raw gaps
  gapsMid.clear();                              // Clear previous mid-gaps

  process_LiDAR(scan_msg, gaps, gapsMid);       // Process scan to fill gaps and gapsMid

  if (!gapsMid.empty()) {                       // If any valid gaps found
    chooseBestGap();                            // Select and publish best gap
  } else {
    RCLCPP_WARN(this->get_logger(), "No sufficient gaps found in scan."); // Warn if none
  }
}

// Trajectory update callback definition
void reactiveColAvoid::traj_callback(const sensor_msgs::msg::trajectories::SharedPtr traj_msg)
{
  latest_traj = *traj_msg;              // Store the received trajectory message
  traj_received_ = true;                // Mark that trajectory has been received
  RCLCPP_INFO(this->get_logger(),      // Log the received angle
              "Trajectory received: theta = %.3f", 
              latest_traj.theta_channel);
}

// LiDAR processing function definition
void reactiveColAvoid::process_LiDAR(
  const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg,             // Input scan
  std::vector<std::pair<double, double>>& gaps,                      // Output raw gaps
  std::vector<std::pair<double, double>>& gapsMid)                   // Output mid-gaps
{
  gaps.clear();        // Clear raw gaps container
  gapsMid.clear();     // Clear mid-gaps container

  double angle_min       = scan_msg->angle_min;      // Start angle of scan
  double angle_increment = scan_msg->angle_increment; // Angular increment per reading
  double min_angle       = -0.7854;                  // -45 degrees in radians
  double max_angle       =  0.7854;                  // +45 degrees in radians

  // Collect gaps where distance > 3.0 within specified angular range
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    double angle    = angle_min + i * angle_increment; // Compute angle
    double distance = scan_msg->ranges[i];            // Read distance
    if (angle >= min_angle && angle <= max_angle && distance > 3.0) {
      gaps.emplace_back(angle, distance);             // Store valid gap
    }
  }

  // Identify contiguous segments of gaps > threshold size and extract midpoint
  for (size_t j = 0; j + 1 < gaps.size(); ++j) {
    size_t start_idx = j;   // Starting index of this gap sequence
    size_t gap_count = 0;   // Count consecutive readings
    while (j + 1 < gaps.size() &&
           std::abs(gaps[j + 1].first - gaps[j].first - angle_increment) < 1e-4)
    {
      ++gap_count;         // Increment count
      ++j;                 // Advance through contiguous gap
    }
    if (gap_count > 38) {  // If large enough gap
      size_t mid_idx = start_idx + gap_count / 2;   // Compute midpoint index
      gapsMid.push_back(gaps[mid_idx]);             // Store midpoint
    }
  }
}

double reactiveColAvoid::velocity()
{
  double speed = 0.0; // Initialize velocity

  //logic to determine velocity based on conditions
  return speed; // Return computed velocity
}

// Choose best gap based on trajectory and publish drive command
void reactiveColAvoid::chooseBestGap()
{
  if (!traj_received_) {                          // If trajectory not yet received
    RCLCPP_WARN(this->get_logger(), "No trajectory received yet.");
    return;
  }

  double desired_angle  = latest_traj.theta_channel; // Desired steering angle
  double best_gap_angle = 0.0;                       // Initialize best gap
  double smallest_diff  = std::numeric_limits<double>::max(); // Init smallest difference

  // Find gap closest to desired angle
  for (const auto& gap : gapsMid) {
    double diff = std::abs(gap.first - desired_angle); // Compute angular difference
    if (diff < smallest_diff) {
      smallest_diff  = diff;         // Update smallest diff
      best_gap_angle = gap.first;    // Update best gap angle
    }
  }

  // Construct and publish drive message
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp         = this->now();            // Timestamp
  drive_msg.drive.steering_angle = best_gap_angle;         // Steering command
  drive_msg.drive.speed          = velocity();                    // Speed command (TO DO: Subscribe to speed)

  drive_pub->publish(drive_msg);                           // Publish drive command
  RCLCPP_INFO(this->get_logger(),                          // Log published command
              "Published drive command: angle = %.3f radians", best_gap_angle);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // Initialize ROS2
  auto node = std::make_shared<reactiveColAvoid>(); // Create node instance
  rclcpp::spin(node);       // Spin the node to process callbacks
  rclcpp::shutdown();       // Shutdown ROS2
  return 0;                 // Exit program
}