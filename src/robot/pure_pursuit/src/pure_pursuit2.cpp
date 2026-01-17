#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <chrono>

#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std;

class PurePursuit : public rclcpp::Node {
public:
    PurePursuit()
    : Node("pure_pursuit"),
      received_pose_(false),
      last_point_(0),
      steering_angle_(0.0),
      throttle_cmd_(0.0),
      current_speed_(0.0),
      have_prev_pos_(false)
    {
        // Load waypoints
        string package_path = ament_index_cpp::get_package_share_directory("pure_pursuit");
        string file_path = package_path + "/assets/autoDriveRaceline_with_vel.csv";

        if (!loadWaypoints(file_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints");
        }

        // ---- Subscribers ----
        ips_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/autodrive/f1tenth_1/ips", 10,
            std::bind(&PurePursuit::ipsCallback, this, std::placeholders::_1));

        speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/speed", 10,
            std::bind(&PurePursuit::speedCallback, this, std::placeholders::_1));

        // ---- Publishers ----
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/throttle_command", 10);

        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/autodrive/f1tenth_1/steering_command", 10);

        // ---- Control loop ----
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PurePursuit::controlLoop, this));
    }

private:
    // ---------------- Waypoints ----------------
    vector<pair<double,double>> waypoints_;
    vector<double> velocities_;
    size_t last_point_;

    // ---------------- ROS ----------------
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ips_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ---------------- Vehicle State ----------------
    double current_x_, current_y_, heading_;
    double prev_x_, prev_y_;
    bool received_pose_;
    bool have_prev_pos_;
    double current_speed_;
    double steering_angle_;
    double throttle_cmd_;
    double target_speed_;

    // ---------------- Callbacks ----------------
    void ipsCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;

        if (have_prev_pos_) {
            double dx = current_x_ - prev_x_;
            double dy = current_y_ - prev_y_;
            if (hypot(dx, dy) > 1e-4) {
                heading_ = atan2(dy, dx);
            }
        }

        prev_x_ = current_x_;
        prev_y_ = current_y_;
        have_prev_pos_ = true;
        received_pose_ = true;
    }

    void speedCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_speed_ = msg->data;
    }

    // ---------------- Control Loop ----------------
    void controlLoop() {
        if (!received_pose_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No pose received yet; skipping control loop.");
            return;
        }

        if (waypoints_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Waypoints are empty; skipping control loop.");
            return;
        }

        double base_speed = velocities_.empty() 
            ? 0.2 
            : velocities_[std::min(last_point_, velocities_.size()-1)];

        double ld = std::clamp(2.0 + 0.3 * base_speed, 1.5, 4.0);
        
        // Compute steering first (curvature computed inside)
        computeSteering(ld);

        // Use curvature / steering_angle_ to limit speed for sharp turns
        double max_lateral_accel = 2.0; // m/s^2, adjust to your robot
        
        // safe_speed = sqrt(a_max / |curvature|), approximate with steering_angle_
        double safe_speed = std::sqrt(max_lateral_accel / std::max(fabs(steering_angle_), 0.0001));

        // Clamp target speed between waypoint speed and safe speed
        double min_speed = 0.2;
        double max_speed = 2.0;

        target_speed_ = std::clamp(base_speed, min_speed, safe_speed);

        computeThrottle();
        publishCommands();

        RCLCPP_INFO(this->get_logger(),
            "Current speed: %.2f | Target speed: %.2f | Throttle: %.2f | Steering angle: %.2f rad",
            current_speed_, target_speed_, throttle_cmd_, steering_angle_);
    }

    // ---------------- Pure Pursuit ----------------
    void computeSteering(double ld) {
        auto goal = getGoalPoint(ld);

        double dx = goal.first  - current_x_;
        double dy = goal.second - current_y_;

        // Transform to vehicle frame
        double x_r =  cos(heading_) * dx + sin(heading_) * dy;
        double y_r = -sin(heading_) * dx + cos(heading_) * dy;

        double curvature = (2.0 * y_r) / (ld * ld);
        double wheelbase = 1.0;

        double desired_steering = atan(curvature * wheelbase);

        // Clamp to car limits
        desired_steering = std::clamp(desired_steering, -0.418, 0.418);

        // Smooth with low-pass filter
        double alpha = 0.2;
        steering_angle_ = alpha * desired_steering + (1.0 - alpha) * steering_angle_;
    }

    void computeThrottle() {
        double kp = 0.6;
        throttle_cmd_ = std::clamp(kp * (target_speed_ - current_speed_), 0.0, 1.0);
    }

    void publishCommands() {
        std_msgs::msg::Float32 t, s;
        t.data = throttle_cmd_;
        s.data = steering_angle_;
        throttle_pub_->publish(t);
        steering_pub_->publish(s);
    }

    // ---------------- Helpers ----------------
    double distance(pair<double,double> a, pair<double,double> b) {
        return hypot(a.first - b.first, a.second - b.second);
    }

    pair<double,double> getGoalPoint(double ld) {
        pair<double,double> car = {current_x_, current_y_};
        for (size_t i = last_point_; i < waypoints_.size(); i++) {
            if (distance(car, waypoints_[i]) >= ld) {
                last_point_ = i;
                return waypoints_[i];
            }
        }
        return waypoints_.back();
    }

    bool loadWaypoints(const string &file) {
        ifstream infile(file);
        if (!infile.is_open()) return false;

        string line;
        getline(infile, line);
        while (getline(infile, line)) {
            stringstream ss(line);
            string s, x, y, psi, kappa, vx, ax;
            if (!getline(ss, s, ',') || !getline(ss, x, ',') ||
                !getline(ss, y, ',') || !getline(ss, psi, ',') ||
                !getline(ss, kappa, ',') || !getline(ss, vx, ',') ||
                !getline(ss, ax, ',')) continue;

            waypoints_.emplace_back(stod(x), stod(y));
            velocities_.emplace_back(stod(vx));
        }
        return true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}



    // ---------------- Goal Point Search ----------------
    // pair<double,double> getGoalPoint(double ld, const geometry_msgs::msg::Point &carPos) {
    //     pair<double,double> car_point = {carPos.x, carPos.y};
    //     pair<double,double> best_point = waypoints_.back();  // fallback

    //     for(size_t i = last_point_; i < waypoints_.size(); i++) {
    //         double dist = pointDistance(car_point, waypoints_[i]);

    //         // First waypoint past the lookahead distance
    //         if (dist >= ld) {
    //             last_point_ = i;
    //             return waypoints_[i];
    //         }
    //         // If we just crossed the ld threshold between i-1 and i
    //         if (i > 0 && pointDistance(car_point, waypoints_[i-1]) < ld && dist > ld) {
    //             auto sol = interpolatedPoint(ld, waypoints_[i], waypoints_[i-1], car_point);
    //             if (sol.first != -1.0) {
    //                 last_point_ = i;
    //                 return sol;
    //             }
    //         }
    //     }

    //     return best_point;
    // }


    // // ---------------- Lookahead Interpolation ----------------
    // pair<double,double> interpolatedPoint(double ld,
    //                                     pair<double,double> p1,
    //                                     pair<double,double> p2,
    //                                     pair<double,double> car) 
    // {
    //     // Convert to car-centered frame for intersection
    //     double x1 = p1.first  - car.first;
    //     double y1 = p1.second - car.second;
    //     double x2 = p2.first  - car.first;
    //     double y2 = p2.second - car.second;

    //     double dx = x2 - x1;
    //     double dy = y2 - y1;

    //     double dr2 = dx*dx + dy*dy;
    //     double D = x1*y2 - x2*y1;

    //     double discriminant = ld*ld * dr2 - D*D;
    //     if (discriminant < 0.0) {
    //         return {-1.0, -1.0};
    //     }

    //     double sqrtD = sqrt(discriminant);
    //     double sgn_dy = (dy >= 0.0) ? 1.0 : -1.0;

    //     // Circle-line intersection solutions (in car frame)
    //     double ix1 = (D * dy + sgn_dy * dx * sqrtD) / dr2;
    //     double iy1 = (-D * dx + fabs(dy) * sqrtD) / dr2;

    //     double ix2 = (D * dy - sgn_dy * dx * sqrtD) / dr2;
    //     double iy2 = (-D * dx - fabs(dy) * sqrtD) / dr2;

    //     // Convert back to world frame
    //     pair<double,double> sol1 = {ix1 + car.first, iy1 + car.second};
    //     pair<double,double> sol2 = {ix2 + car.first, iy2 + car.second};

    //     // Choose the one that lies between p1 and p2
    //     if (checkSegment(sol1, p1, p2)) return sol1;
    //     if (checkSegment(sol2, p1, p2)) return sol2;

    //     return {-1.0, -1.0};
    // }

    // ---------------- Utilities ----------------
    // double pointDistance(pair<double,double> a, pair<double,double> b) {
    //     return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
    // }

    // bool checkSegment(pair<double,double> p,
    //                 pair<double,double> a,
    //                 pair<double,double> b)
    // {
    //     double minX = min(a.first,  b.first);
    //     double maxX = max(a.first,  b.first);
    //     double minY = min(a.second, b.second);
    //     double maxY = max(a.second, b.second);

    //     return (p.first >= minX && p.first <= maxX &&
    //             p.second >= minY && p.second <= maxY);
    // }