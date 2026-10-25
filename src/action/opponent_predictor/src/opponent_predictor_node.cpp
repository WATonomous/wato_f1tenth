#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

constexpr double kEps = 1e-9;

double wrapAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.z = std::sin(0.5 * yaw);
  q.x = 0.0;
  q.y = 0.0;
  return q;
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ';')) {
    fields.push_back(field);
  }
  return fields;
}

struct Waypoint
{
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};

struct TrackPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};

struct Projection
{
  double s = 0.0;
  double lateral_error = 0.0;
  double distance = 0.0;
  double yaw = 0.0;
  double profile_speed = 0.0;
};

class OpponentPredictorNode : public rclcpp::Node
{
public:
  OpponentPredictorNode()
  : Node("opponent_predictor_node")
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/opp_racecar/odom");
    estimated_odom_topic_ = declare_parameter<std::string>("estimated_odom_topic", "/opponent/odom");
    predicted_path_topic_ = declare_parameter<std::string>("predicted_path_topic", "/opponent/predicted_path");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/opponent/markers");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/opponent/debug");
    waypoint_path_ = declare_parameter<std::string>("waypoint_path", "");
    waypoint_path_absolute_ = declare_parameter<bool>("waypoint_path_absolute", true);
    reverse_waypoints_ = declare_parameter<bool>("reverse_waypoints", false);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    pose_source_ = declare_parameter<std::string>("pose_source", "odom_pose");
    tf_lookup_timeout_ = std::max(0.0, declare_parameter<double>("tf_lookup_timeout", 0.02));
    prediction_steps_ = std::max(1, static_cast<int>(declare_parameter<int>("prediction_steps", 5)));
    prediction_dt_ = std::max(1e-3, declare_parameter<double>("prediction_dt", 0.1));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));
    stale_timeout_ = std::max(0.0, declare_parameter<double>("stale_timeout", 0.5));
    max_stale_prediction_time_ =
      std::max(stale_timeout_, declare_parameter<double>("max_stale_prediction_time", 2.0));
    speed_profile_scale_ = std::max(0.0, declare_parameter<double>("speed_profile_scale", 1.0));
    speed_profile_min_speed_ = std::max(0.0, declare_parameter<double>("speed_profile_min_speed", 0.0));
    speed_profile_max_speed_ = std::max(
      speed_profile_min_speed_,
      declare_parameter<double>("speed_profile_max_speed", 20.0));
    profile_speed_blend_ = std::clamp(
      declare_parameter<double>("profile_speed_blend", 0.7), 0.0, 1.0);
    out_of_sight_profile_speed_blend_ = std::clamp(
      declare_parameter<double>("out_of_sight_profile_speed_blend", 1.0), 0.0, 1.0);
    use_profile_speed_fallback_ = declare_parameter<bool>("use_profile_speed_fallback", false);
    stationary_speed_threshold_ = std::max(
      0.0, declare_parameter<double>("stationary_speed_threshold", 0.15));
    hold_stationary_when_stale_ = declare_parameter<bool>("hold_stationary_when_stale", true);
    lateral_offset_alpha_ = std::clamp(
      declare_parameter<double>("lateral_offset_alpha", 0.35), 0.0, 1.0);
    lateral_offset_decay_time_ = std::max(
      1e-3, declare_parameter<double>("lateral_offset_decay_time", 0.3));
    position_snap_alpha_ = std::clamp(
      declare_parameter<double>("position_snap_alpha", 1.0), 0.0, 1.0);
    initial_speed_ = std::max(0.0, declare_parameter<double>("initial_speed", 1.0));
    max_progress_speed_ = std::max(0.1, declare_parameter<double>("max_progress_speed", 12.0));
    max_progress_accel_ = std::max(0.0, declare_parameter<double>("max_progress_accel", 8.0));

    q_s_ = std::max(0.0, declare_parameter<double>("kf_process_var_s", 0.05));
    q_v_ = std::max(0.0, declare_parameter<double>("kf_process_var_v", 0.5));
    r_s_ = std::max(1e-6, declare_parameter<double>("kf_measurement_var_s", 0.15));
    r_v_ = std::max(1e-6, declare_parameter<double>("kf_measurement_var_v", 0.6));

    loadWaypoints(resolveWaypointPath(waypoint_path_, waypoint_path_absolute_));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_,
      sensor_qos,
      std::bind(&OpponentPredictorNode::odomCallback, this, std::placeholders::_1));

    estimated_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(estimated_odom_topic_, sensor_qos);
    predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>(predicted_path_topic_, sensor_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, debug_qos);
    debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(debug_topic_, debug_qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&OpponentPredictorNode::timerCallback, this));

    // Live params refreshed at 2 Hz instead of on every odom/timer callback
    // (was ~30 Hz x ~25 mutex-protected param fetches).
    params_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&OpponentPredictorNode::refreshLiveParams, this));

    RCLCPP_INFO(
      get_logger(),
      "Opponent predictor ready: topic=%s, waypoints=%zu, horizon=%d x %.3fs",
      input_odom_topic_.c_str(),
      waypoints_.size(),
      prediction_steps_,
      prediction_dt_);
  }

private:
  void refreshLiveParams()
  {
    pose_source_ = get_parameter("pose_source").as_string();
    tf_lookup_timeout_ = std::max(0.0, get_parameter("tf_lookup_timeout").as_double());
    prediction_steps_ = std::max(1, static_cast<int>(get_parameter("prediction_steps").as_int()));
    prediction_dt_ = std::max(1e-3, get_parameter("prediction_dt").as_double());
    stale_timeout_ = std::max(0.0, get_parameter("stale_timeout").as_double());
    max_stale_prediction_time_ = std::max(
      stale_timeout_,
      get_parameter("max_stale_prediction_time").as_double());
    speed_profile_scale_ = std::max(0.0, get_parameter("speed_profile_scale").as_double());
    speed_profile_min_speed_ = std::max(0.0, get_parameter("speed_profile_min_speed").as_double());
    speed_profile_max_speed_ = std::max(
      speed_profile_min_speed_,
      get_parameter("speed_profile_max_speed").as_double());
    profile_speed_blend_ = std::clamp(
      get_parameter("profile_speed_blend").as_double(), 0.0, 1.0);
    out_of_sight_profile_speed_blend_ = std::clamp(
      get_parameter("out_of_sight_profile_speed_blend").as_double(), 0.0, 1.0);
    use_profile_speed_fallback_ = get_parameter("use_profile_speed_fallback").as_bool();
    stationary_speed_threshold_ = std::max(
      0.0, get_parameter("stationary_speed_threshold").as_double());
    hold_stationary_when_stale_ = get_parameter("hold_stationary_when_stale").as_bool();
    lateral_offset_alpha_ = std::clamp(
      get_parameter("lateral_offset_alpha").as_double(), 0.0, 1.0);
    lateral_offset_decay_time_ = std::max(
      1e-3,
      get_parameter("lateral_offset_decay_time").as_double());
    position_snap_alpha_ = std::clamp(
      get_parameter("position_snap_alpha").as_double(), 0.0, 1.0);
    max_progress_speed_ = std::max(0.1, get_parameter("max_progress_speed").as_double());
    max_progress_accel_ = std::max(0.0, get_parameter("max_progress_accel").as_double());
    q_s_ = std::max(0.0, get_parameter("kf_process_var_s").as_double());
    q_v_ = std::max(0.0, get_parameter("kf_process_var_v").as_double());
    r_s_ = std::max(1e-6, get_parameter("kf_measurement_var_s").as_double());
    r_v_ = std::max(1e-6, get_parameter("kf_measurement_var_v").as_double());
  }

  std::string resolveWaypointPath(const std::string & path, bool absolute)
  {
    if (path.empty() || absolute || (!path.empty() && path.front() == '/')) {
      return path;
    }
    const auto share_dir = ament_index_cpp::get_package_share_directory("mppi_bringup");
    return share_dir + "/" + path;
  }

  void loadWaypoints(const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "No waypoint_path supplied; opponent predictor will not publish.");
      return;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open opponent predictor waypoint_path: %s", path.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      const auto fields = splitCsvLine(line);
      if (fields.size() < 6) {
        continue;
      }
      try {
        Waypoint wp;
        wp.s = std::stod(fields[0]);
        wp.x = std::stod(fields[1]);
        wp.y = std::stod(fields[2]);
        wp.yaw = std::stod(fields[3]);
        wp.speed = std::stod(fields[5]);
        waypoints_.push_back(wp);
      } catch (const std::exception &) {
        continue;
      }
    }

    if (waypoints_.size() < 2) {
      RCLCPP_ERROR(get_logger(), "Waypoint file has fewer than two usable points: %s", path.c_str());
      waypoints_.clear();
      return;
    }

    if (reverse_waypoints_) {
      reverseWaypoints();
      RCLCPP_INFO(get_logger(), "reverse_waypoints=true: centerline flipped to match opposite travel direction.");
    }

    const auto & first = waypoints_.front();
    const auto & last = waypoints_.back();
    track_length_ = last.s + std::hypot(first.x - last.x, first.y - last.y);
    if (track_length_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid track length from waypoint_path: %s", path.c_str());
      waypoints_.clear();
      return;
    }
  }

  // Reverse the loaded centerline so its arc-length s increases *with* travel
  // when racing the track in the opposite direction. Without this the opponent
  // moves toward decreasing s: progress speed comes out negative (and is
  // rejected by the validity gate), the horizon propagates the wrong way around
  // the loop, and predicted headings point backwards. Flipping the point order,
  // recomputing s as cumulative chord length, and rotating each heading by pi
  // keeps projection, the Kalman progress speed, prediction, and yaws all
  // self-consistent. Applied once at load time; not a live param.
  void reverseWaypoints()
  {
    std::reverse(waypoints_.begin(), waypoints_.end());
    waypoints_.front().s = 0.0;
    waypoints_.front().yaw = wrapAngle(waypoints_.front().yaw + M_PI);
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
      const double ds = std::hypot(
        waypoints_[i].x - waypoints_[i - 1].x,
        waypoints_[i].y - waypoints_[i - 1].y);
      waypoints_[i].s = waypoints_[i - 1].s + ds;
      waypoints_[i].yaw = wrapAngle(waypoints_[i].yaw + M_PI);
    }
  }

  double normalizeS(double s) const
  {
    if (track_length_ <= 0.0) {
      return s;
    }
    s = std::fmod(s, track_length_);
    if (s < 0.0) {
      s += track_length_;
    }
    return s;
  }

  double unwrapS(double measured_s, double reference_s) const
  {
    if (track_length_ <= 0.0) {
      return measured_s;
    }
    double unwrapped = measured_s;
    while (unwrapped - reference_s > 0.5 * track_length_) {
      unwrapped -= track_length_;
    }
    while (unwrapped - reference_s < -0.5 * track_length_) {
      unwrapped += track_length_;
    }
    return unwrapped;
  }

  TrackPose interpolateTrack(double s) const
  {
    TrackPose pose;
    if (waypoints_.empty()) {
      return pose;
    }

    const double s_norm = normalizeS(s);
    std::size_t idx = waypoints_.size() - 1;
    for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
      if (s_norm >= waypoints_[i].s && s_norm < waypoints_[i + 1].s) {
        idx = i;
        break;
      }
    }

    const Waypoint & a = waypoints_[idx];
    const Waypoint & b = (idx + 1 < waypoints_.size()) ? waypoints_[idx + 1] : waypoints_.front();
    const double b_s = (idx + 1 < waypoints_.size()) ? b.s : track_length_;
    const double segment_len = std::max(kEps, b_s - a.s);
    const double ratio = std::clamp((s_norm - a.s) / segment_len, 0.0, 1.0);

    pose.x = a.x + ratio * (b.x - a.x);
    pose.y = a.y + ratio * (b.y - a.y);
    const double yaw_delta = wrapAngle(b.yaw - a.yaw);
    pose.yaw = wrapAngle(a.yaw + ratio * yaw_delta);
    const double raw_speed = a.speed + ratio * (b.speed - a.speed);
    pose.speed = std::clamp(raw_speed * speed_profile_scale_, speed_profile_min_speed_, speed_profile_max_speed_);
    return pose;
  }

  TrackPose applyLateralOffset(const TrackPose & centerline_pose, double lateral_offset) const
  {
    TrackPose pose = centerline_pose;
    pose.x += -std::sin(centerline_pose.yaw) * lateral_offset;
    pose.y += std::cos(centerline_pose.yaw) * lateral_offset;
    return pose;
  }

  Projection projectToTrack(double x, double y) const
  {
    Projection best;
    best.distance = std::numeric_limits<double>::infinity();
    if (waypoints_.size() < 2) {
      return best;
    }

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      const Waypoint & a = waypoints_[i];
      const Waypoint & b = (i + 1 < waypoints_.size()) ? waypoints_[i + 1] : waypoints_.front();
      const double b_s = (i + 1 < waypoints_.size()) ? b.s : track_length_;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len2 = std::max(kEps, dx * dx + dy * dy);
      const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len2, 0.0, 1.0);
      const double px = a.x + t * dx;
      const double py = a.y + t * dy;
      const double ex = x - px;
      const double ey = y - py;
      const double dist = std::hypot(ex, ey);

      if (dist < best.distance) {
        best.distance = dist;
        best.s = normalizeS(a.s + t * (b_s - a.s));
        best.yaw = std::atan2(dy, dx);
        best.profile_speed = interpolateTrack(best.s).speed;
        const double nx = -std::sin(best.yaw);
        const double ny = std::cos(best.yaw);
        best.lateral_error = ex * nx + ey * ny;
      }
    }
    return best;
  }

  void predictKalmanTo(double now_sec)
  {
    if (!initialized_) {
      return;
    }
    const double dt = std::max(0.0, now_sec - last_filter_time_sec_);
    if (dt <= 0.0) {
      return;
    }

    s_hat_ += v_hat_ * dt;
    p00_ = p00_ + dt * (p10_ + p01_) + dt * dt * p11_ + q_s_;
    p01_ = p01_ + dt * p11_;
    p10_ = p10_ + dt * p11_;
    p11_ = p11_ + q_v_;
    last_filter_time_sec_ = now_sec;
  }

  void updateKalman(double s_meas, double v_meas)
  {
    // Sequential scalar measurement updates for z = [s, v].
    const double y_s = s_meas - s_hat_;
    const double S_s = p00_ + r_s_;
    const double K0_s = p00_ / S_s;
    const double K1_s = p10_ / S_s;
    s_hat_ += K0_s * y_s;
    v_hat_ += K1_s * y_s;
    const double old_p00 = p00_;
    const double old_p01 = p01_;
    p00_ -= K0_s * old_p00;
    p01_ -= K0_s * old_p01;
    p10_ -= K1_s * old_p00;
    p11_ -= K1_s * old_p01;

    const double y_v = v_meas - v_hat_;
    const double S_v = p11_ + r_v_;
    const double K0_v = p01_ / S_v;
    const double K1_v = p11_ / S_v;
    s_hat_ += K0_v * y_v;
    v_hat_ += K1_v * y_v;
    const double old2_p10 = p10_;
    const double old2_p11 = p11_;
    p00_ -= K0_v * old2_p10;
    p01_ -= K0_v * old2_p11;
    p10_ -= K1_v * old2_p10;
    p11_ -= K1_v * old2_p11;

    v_hat_ = std::max(0.0, v_hat_);
  }

  bool transformOpponentPose(
    const nav_msgs::msg::Odometry & msg,
    geometry_msgs::msg::PoseStamped & pose_out)
  {
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = msg.header;
    if (pose_in.header.frame_id.empty()) {
      pose_in.header.frame_id = frame_id_;
    }
    pose_in.pose = msg.pose.pose;

    if (pose_source_ == "tf" && !msg.child_frame_id.empty()) {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          frame_id_,
          msg.child_frame_id,
          tf2::TimePointZero,
          tf2::durationFromSec(tf_lookup_timeout_));
        pose_out.header = transform.header;
        pose_out.pose.position.x = transform.transform.translation.x;
        pose_out.pose.position.y = transform.transform.translation.y;
        pose_out.pose.position.z = transform.transform.translation.z;
        pose_out.pose.orientation = transform.transform.rotation;
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Could not lookup opponent TF from '%s' to '%s': %s",
          msg.child_frame_id.c_str(),
          frame_id_.c_str(),
          ex.what());
        return false;
      }
    }

    if (pose_source_ == "odom_pose" && pose_in.header.frame_id == frame_id_) {
      pose_out = pose_in;
      return true;
    }

    if (pose_source_ == "odom_pose" && pose_in.header.frame_id.empty()) {
      pose_in.header.frame_id = frame_id_;
      pose_out = pose_in;
      return true;
    }

    try {
      pose_out = tf_buffer_->transform(
        pose_in,
        frame_id_,
        tf2::durationFromSec(tf_lookup_timeout_));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Could not transform opponent odom from '%s' to '%s': %s",
        pose_in.header.frame_id.c_str(),
        frame_id_.c_str(),
        ex.what());
      return false;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // refreshLiveParams() moved to params_timer_ (2 Hz) — see ctor.
    if (waypoints_.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped opponent_pose;
    if (!transformOpponentPose(*msg, opponent_pose)) {
      return;
    }

    const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    const double now_sec = stamp_sec > 0.0 ? stamp_sec : now().seconds();
    const double x = opponent_pose.pose.position.x;
    const double y = opponent_pose.pose.position.y;
    if (!std::isfinite(now_sec) || !std::isfinite(x) || !std::isfinite(y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Ignoring non-finite opponent odom measurement.");
      return;
    }
    const Projection proj = projectToTrack(x, y);

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double vz = msg->twist.twist.linear.z;
    last_twist_speed_ = (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) ?
      std::hypot(std::hypot(vx, vy), vz) :
      std::numeric_limits<double>::quiet_NaN();

    double s_meas = proj.s;
    double v_meas = last_twist_speed_;
    last_progress_speed_ = std::numeric_limits<double>::quiet_NaN();
    bool progress_speed_valid = false;

    if (initialized_) {
      s_meas = unwrapS(proj.s, s_hat_);
      const double s_for_delta = unwrapS(proj.s, last_projected_s_);
      const double dt = now_sec - last_projection_time_sec_;
      if (dt > 1e-3) {
        const double raw_progress_speed = (s_for_delta - last_projected_s_) / dt;
        last_progress_speed_ = raw_progress_speed;
        progress_speed_valid =
          std::isfinite(raw_progress_speed) &&
          raw_progress_speed >= -0.25 &&
          raw_progress_speed <= max_progress_speed_;

        if (progress_speed_valid) {
          v_meas = std::clamp(raw_progress_speed, 0.0, max_progress_speed_);
          if (std::isfinite(last_measured_speed_) && max_progress_accel_ > 0.0) {
            const double max_step = max_progress_accel_ * dt;
            v_meas = std::clamp(
              v_meas,
              std::max(0.0, last_measured_speed_ - max_step),
              last_measured_speed_ + max_step);
          }
        }
      }
    }

    if (
      use_profile_speed_fallback_ &&
      !progress_speed_valid &&
      (!std::isfinite(v_meas) || std::abs(v_meas) < 0.05))
    {
      v_meas = proj.profile_speed;
    }
    v_meas = std::clamp(v_meas, 0.0, max_progress_speed_);

    if (!initialized_) {
      initialized_ = true;
      s_hat_ = proj.s;
      v_hat_ = std::isfinite(v_meas) ? v_meas : initial_speed_;
      lateral_offset_hat_ = proj.lateral_error;
      last_filter_time_sec_ = now_sec;
    } else {
      predictKalmanTo(now_sec);
      updateKalman(s_meas, v_meas);
      s_hat_ = (1.0 - position_snap_alpha_) * s_hat_ + position_snap_alpha_ * s_meas;
      lateral_offset_hat_ =
        (1.0 - lateral_offset_alpha_) * lateral_offset_hat_ +
        lateral_offset_alpha_ * proj.lateral_error;
    }

    last_measurement_time_sec_ = now_sec;
    last_projection_time_sec_ = now_sec;
    last_projected_s_ = initialized_ ? unwrapS(proj.s, s_hat_) : proj.s;
    last_measured_speed_ = v_meas;
    last_projection_distance_ = proj.distance;
    last_lateral_error_ = proj.lateral_error;
    last_profile_speed_ = proj.profile_speed;
  }

  std::vector<TrackPose> makePrediction(double now_sec, bool stale) const
  {
    std::vector<TrackPose> prediction;
    prediction.reserve(static_cast<std::size_t>(prediction_steps_ + 1));

    double pred_s = s_hat_;
    double pred_v = std::max(0.0, v_hat_);
    if (pred_v < stationary_speed_threshold_) {
      pred_v = 0.0;
    }
    double blend = stale ? out_of_sight_profile_speed_blend_ : profile_speed_blend_;
    const bool stationary = pred_v <= stationary_speed_threshold_;
    if (stationary && (!stale || hold_stationary_when_stale_)) {
      blend = 0.0;
    }

    for (int k = 0; k <= prediction_steps_; ++k) {
      TrackPose pose = interpolateTrack(pred_s);
      const double lateral_decay = std::exp(
        -static_cast<double>(k) * prediction_dt_ / lateral_offset_decay_time_);
      pose = applyLateralOffset(pose, lateral_offset_hat_ * lateral_decay);
      pose.speed = pred_v;
      prediction.push_back(pose);

      const double profile_v = interpolateTrack(pred_s).speed;
      pred_v = (1.0 - blend) * pred_v + blend * profile_v;
      if (stationary && (!stale || hold_stationary_when_stale_)) {
        pred_v = 0.0;
      }
      pred_s += pred_v * prediction_dt_;
    }

    (void)now_sec;
    return prediction;
  }

  void publishPrediction(const std::vector<TrackPose> & prediction, bool stale, double age)
  {
    if (prediction.empty()) {
      return;
    }

    const auto stamp = now();
    const TrackPose & current = prediction.front();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = "opponent_estimate";
    odom.pose.pose.position.x = current.x;
    odom.pose.pose.position.y = current.y;
    odom.pose.pose.orientation = yawToQuaternion(current.yaw);
    odom.twist.twist.linear.x = current.speed;
    estimated_odom_pub_->publish(odom);

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = frame_id_;
    for (const auto & p : prediction) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation = yawToQuaternion(p.yaw);
      path.poses.push_back(ps);
    }
    predicted_path_pub_->publish(path);

    visualization_msgs::msg::MarkerArray markers;
    addDeleteAllMarker(markers);
    addPathLineMarker(markers, prediction, stamp, stale);
    addCurrentMarker(markers, current, stamp, stale);
    addHorizonPointMarkers(markers, prediction, stamp, stale);
    addTextMarker(markers, current, stamp, stale, age);
    marker_pub_->publish(markers);
    prediction_visualization_cleared_ = false;

    std_msgs::msg::Float32MultiArray debug;
    debug.data = {
      static_cast<float>(normalizeS(last_projected_s_)),
      static_cast<float>(v_hat_),
      static_cast<float>(last_progress_speed_),
      static_cast<float>(last_twist_speed_),
      static_cast<float>(last_profile_speed_),
      static_cast<float>(last_projection_distance_),
      static_cast<float>(lateral_offset_hat_),
      stale ? 1.0f : 0.0f
    };
    debug_pub_->publish(debug);
  }

  void clearPredictionVisualization()
  {
    if (prediction_visualization_cleared_) {
      return;
    }

    const auto stamp = now();

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = frame_id_;
    predicted_path_pub_->publish(path);

    visualization_msgs::msg::MarkerArray markers;
    addDeleteAllMarker(markers);
    marker_pub_->publish(markers);

    prediction_visualization_cleared_ = true;
  }

  void addDeleteAllMarker(visualization_msgs::msg::MarkerArray & markers) const
  {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
  }

  void addPathLineMarker(
    visualization_msgs::msg::MarkerArray & markers,
    const std::vector<TrackPose> & prediction,
    const rclcpp::Time & stamp,
    bool stale) const
  {
    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = frame_id_;
    line.ns = "opponent_prediction";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.06;
    line.color.a = 0.9;
    line.color.r = stale ? 1.0 : 0.1;
    line.color.g = stale ? 0.55 : 0.8;
    line.color.b = stale ? 0.1 : 1.0;
    for (const auto & p : prediction) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0.08;
      line.points.push_back(point);
    }
    markers.markers.push_back(line);
  }

  void addCurrentMarker(
    visualization_msgs::msg::MarkerArray & markers,
    const TrackPose & current,
    const rclcpp::Time & stamp,
    bool stale) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id_;
    marker.ns = "opponent_prediction";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = current.x;
    marker.pose.position.y = current.y;
    marker.pose.position.z = 0.12;
    marker.pose.orientation = yawToQuaternion(current.yaw);
    marker.scale.x = 0.4;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.a = 0.95;
    marker.color.r = stale ? 1.0 : 0.05;
    marker.color.g = stale ? 0.45 : 0.85;
    marker.color.b = stale ? 0.05 : 0.25;
    markers.markers.push_back(marker);
  }

  void addHorizonPointMarkers(
    visualization_msgs::msg::MarkerArray & markers,
    const std::vector<TrackPose> & prediction,
    const rclcpp::Time & stamp,
    bool stale) const
  {
    visualization_msgs::msg::Marker points;
    points.header.stamp = stamp;
    points.header.frame_id = frame_id_;
    points.ns = "opponent_prediction";
    points.id = 2;
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = 0.16;
    points.scale.y = 0.16;
    points.scale.z = 0.16;
    points.color.a = 0.85;
    points.color.r = stale ? 1.0 : 0.0;
    points.color.g = stale ? 0.8 : 0.3;
    points.color.b = stale ? 0.0 : 1.0;
    for (const auto & p : prediction) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0.1;
      points.points.push_back(point);
    }
    markers.markers.push_back(points);
  }

  void addTextMarker(
    visualization_msgs::msg::MarkerArray & markers,
    const TrackPose & current,
    const rclcpp::Time & stamp,
    bool stale,
    double age) const
  {
    visualization_msgs::msg::Marker text;
    text.header.stamp = stamp;
    text.header.frame_id = frame_id_;
    text.ns = "opponent_prediction";
    text.id = 3;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = current.x;
    text.pose.position.y = current.y;
    text.pose.position.z = 0.7;
    text.scale.z = 0.22;
    text.color.a = 0.95;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    std::ostringstream ss;
    ss << "v=" << std::fixed << std::setprecision(1) << v_hat_
       << " age=" << age
       << (stale ? " stale" : "");
    text.text = ss.str();
    markers.markers.push_back(text);
  }

  void timerCallback()
  {
    // refreshLiveParams() moved to params_timer_ (2 Hz) — see ctor.
    if (!initialized_ || waypoints_.empty()) {
      return;
    }

    const double now_sec = now().seconds();
    predictKalmanTo(now_sec);
    const double age = now_sec - last_measurement_time_sec_;
    if (age > stale_timeout_) {
      clearPredictionVisualization();
      return;
    }
    publishPrediction(makePrediction(now_sec, false), false, age);
  }

  std::string input_odom_topic_;
  std::string estimated_odom_topic_;
  std::string predicted_path_topic_;
  std::string marker_topic_;
  std::string debug_topic_;
  std::string waypoint_path_;
  bool waypoint_path_absolute_ = true;
  bool reverse_waypoints_ = false;
  std::string frame_id_ = "map";
  std::string pose_source_ = "odom_pose";
  double tf_lookup_timeout_ = 0.02;
  int prediction_steps_ = 5;
  double prediction_dt_ = 0.1;
  double publish_rate_hz_ = 20.0;
  double stale_timeout_ = 0.5;
  double max_stale_prediction_time_ = 2.0;
  double speed_profile_scale_ = 1.0;
  double speed_profile_min_speed_ = 0.0;
  double speed_profile_max_speed_ = 20.0;
  double profile_speed_blend_ = 0.7;
  double out_of_sight_profile_speed_blend_ = 1.0;
  bool use_profile_speed_fallback_ = false;
  double stationary_speed_threshold_ = 0.15;
  bool hold_stationary_when_stale_ = true;
  double lateral_offset_alpha_ = 0.35;
  double lateral_offset_decay_time_ = 1.0;
  double position_snap_alpha_ = 1.0;
  double initial_speed_ = 1.0;
  double max_progress_speed_ = 12.0;
  double max_progress_accel_ = 8.0;
  double q_s_ = 0.05;
  double q_v_ = 0.5;
  double r_s_ = 0.15;
  double r_v_ = 0.6;

  std::vector<Waypoint> waypoints_;
  double track_length_ = 0.0;

  bool initialized_ = false;
  bool prediction_visualization_cleared_ = true;
  double s_hat_ = 0.0;
  double v_hat_ = 0.0;
  double p00_ = 0.5;
  double p01_ = 0.0;
  double p10_ = 0.0;
  double p11_ = 1.0;
  double last_filter_time_sec_ = 0.0;
  double last_measurement_time_sec_ = -std::numeric_limits<double>::infinity();
  double last_projection_time_sec_ = -std::numeric_limits<double>::infinity();
  double last_projected_s_ = 0.0;
  double last_measured_speed_ = std::numeric_limits<double>::quiet_NaN();
  double last_progress_speed_ = std::numeric_limits<double>::quiet_NaN();
  double last_twist_speed_ = std::numeric_limits<double>::quiet_NaN();
  double last_profile_speed_ = std::numeric_limits<double>::quiet_NaN();
  double last_projection_distance_ = 0.0;
  double last_lateral_error_ = 0.0;
  double lateral_offset_hat_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr params_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpponentPredictorNode>());
  rclcpp::shutdown();
  return 0;
}