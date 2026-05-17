#include "local_planner/local_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

LocalPlannerNode::LocalPlannerNode()
    : Node("local_planner_node"),
      have_odom_(false),
      have_scan_(false),
      debug_map_loaded_(false),
      debug_map_published_(false) {
    init_parameters();

    core_ = std::make_unique<LocalPlannerCore>(core_params_);

    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(local_path_topic_, 10);
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(ready_topic_, latched_qos);
    debug_active_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/local_planner/debug/active_path", 10);
    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/local_planner/debug/markers", latched_qos);
    debug_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/local_planner/debug/map", latched_qos);

    publish_track_map();

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_path_topic_,
        latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
            global_path_ = *msg;
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            odom_ = *msg;
            have_odom_ = true;
        });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            scan_ = *msg;
            have_scan_ = true;
        });

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic_,
        10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            clicked_point_callback(msg);
        });

    clear_manual_obstacles_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        clear_manual_obstacles_topic_,
        10,
        [this](const std_msgs::msg::Empty::SharedPtr msg) {
            clear_manual_obstacles_callback(msg);
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    plan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            plan_timer_callback();
        });
}

void LocalPlannerNode::init_parameters() {
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("local_frame_id", "base_link");
    this->declare_parameter<std::string>("global_path_topic", "/global_planner/path");
    this->declare_parameter<std::string>("local_path_topic", "/local_path");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("ready_topic", "/overtake_ready");
    this->declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    this->declare_parameter<std::string>("clear_manual_obstacles_topic", "/local_planner/debug/clear_obstacles");
    this->declare_parameter<bool>("use_scan_obstacles", false);
    this->declare_parameter<bool>("use_straight_path", true);
    this->declare_parameter<bool>("debug_map_enabled", true);
    this->declare_parameter<std::string>("debug_map_package", "global_planner");
    this->declare_parameter<std::string>("debug_map_file", "/assets/my_map_clean.csv");

    this->declare_parameter<double>("planning_distance", 4.0);
    this->declare_parameter<double>("goal_lookahead", 2.0);
    this->declare_parameter<std::vector<double>>(
        "goal_retry_lookaheads", std::vector<double>{1.6, 1.2, 0.9, 0.6});
    this->declare_parameter<double>("path_speed", 2.5);
    this->declare_parameter<double>("max_lateral", 2.2);
    this->declare_parameter<double>("step_size", 0.20);
    this->declare_parameter<double>("goal_tolerance", 0.45);
    this->declare_parameter<double>("obstacle_inflation", 0.35);
    this->declare_parameter<double>("scan_min_range", 0.0);
    this->declare_parameter<double>("scan_max_range", 5.0);
    this->declare_parameter<double>("scan_front_min_x", 0.0);
    this->declare_parameter<double>("laser_to_base_x", 0.27);
    this->declare_parameter<double>("laser_to_base_y", 0.0);
    this->declare_parameter<double>("manual_obstacle_lifetime", 30.0);
    this->declare_parameter<int>("max_manual_obstacles", 32);
    this->declare_parameter<int>("max_iterations", 3000);
    this->declare_parameter<int>("plan_points", 24);
    this->declare_parameter<double>("goal_sample_rate", 0.40);

    global_frame_id_ = this->get_parameter("global_frame_id").as_string();
    global_path_topic_ = this->get_parameter("global_path_topic").as_string();
    local_path_topic_ = this->get_parameter("local_path_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    ready_topic_ = this->get_parameter("ready_topic").as_string();
    clicked_point_topic_ = this->get_parameter("clicked_point_topic").as_string();
    clear_manual_obstacles_topic_ = this->get_parameter("clear_manual_obstacles_topic").as_string();
    use_scan_obstacles_ = this->get_parameter("use_scan_obstacles").as_bool();
    debug_map_enabled_ = this->get_parameter("debug_map_enabled").as_bool();
    debug_map_package_ = this->get_parameter("debug_map_package").as_string();
    debug_map_file_ = this->get_parameter("debug_map_file").as_string();
    goal_lookahead_ = this->get_parameter("goal_lookahead").as_double();
    goal_retry_lookaheads_ = this->get_parameter("goal_retry_lookaheads").as_double_array();
    manual_obstacle_lifetime_ = this->get_parameter("manual_obstacle_lifetime").as_double();
    const auto max_manual_obstacles = this->get_parameter("max_manual_obstacles").as_int();
    max_manual_obstacles_ = max_manual_obstacles > 0 ? static_cast<size_t>(max_manual_obstacles) : 0U;

    core_params_.use_straight_path = this->get_parameter("use_straight_path").as_bool();
    core_params_.local_frame_id = this->get_parameter("local_frame_id").as_string();
    core_params_.planning_distance = this->get_parameter("planning_distance").as_double();
    core_params_.path_speed = this->get_parameter("path_speed").as_double();
    core_params_.max_lateral = this->get_parameter("max_lateral").as_double();
    core_params_.step_size = this->get_parameter("step_size").as_double();
    core_params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    core_params_.obstacle_inflation = this->get_parameter("obstacle_inflation").as_double();
    core_params_.scan_min_range = this->get_parameter("scan_min_range").as_double();
    core_params_.scan_max_range = this->get_parameter("scan_max_range").as_double();
    core_params_.scan_front_min_x = this->get_parameter("scan_front_min_x").as_double();
    core_params_.laser_to_base_x = this->get_parameter("laser_to_base_x").as_double();
    core_params_.laser_to_base_y = this->get_parameter("laser_to_base_y").as_double();
    core_params_.max_iterations = this->get_parameter("max_iterations").as_int();
    core_params_.plan_points = this->get_parameter("plan_points").as_int();
    core_params_.goal_sample_rate = this->get_parameter("goal_sample_rate").as_double();
}

std::vector<double> LocalPlannerNode::build_goal_ladder() const {
    std::vector<double> ladder;
    ladder.reserve(goal_retry_lookaheads_.size() + 1);

    auto append_unique = [&](double lookahead) {
        if (!std::isfinite(lookahead) || lookahead <= 0.0) {
            return;
        }
        for (const auto existing : ladder) {
            if (std::abs(existing - lookahead) < 1e-6) {
                return;
            }
        }
        ladder.push_back(lookahead);
    };

    append_unique(goal_lookahead_);
    for (const auto lookahead : goal_retry_lookaheads_) {
        append_unique(lookahead);
    }

    if (ladder.empty()) {
        ladder.push_back(core_params_.step_size);
    }

    return ladder;
}

void LocalPlannerNode::plan_timer_callback() {
    if (!have_odom_ || global_path_.poses.empty()) {
        publish_ready(false);
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            core_params_.local_frame_id, global_frame_id_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Could not transform %s to %s: %s",
            global_frame_id_.c_str(),
            core_params_.local_frame_id.c_str(),
            ex.what());
        publish_ready(false);
        return;
    }

    publish_track_map();

    ScanFrame scan_frame;
    if (use_scan_obstacles_ && have_scan_) {
        scan_frame.ranges.assign(scan_.ranges.begin(), scan_.ranges.end());
        scan_frame.angle_min = scan_.angle_min;
        scan_frame.angle_increment = scan_.angle_increment;
        scan_frame.range_min = scan_.range_min;
        scan_frame.range_max = scan_.range_max;
    }

    const auto manual_obstacles_local = manual_obstacles_to_local(transform);
    const auto lookahead_ladder = build_goal_ladder();
    std::optional<geometry_msgs::msg::Point> selected_goal;
    std::optional<geometry_msgs::msg::Point> last_attempt_goal;
    std::optional<LocalPath> selected_path;
    double selected_lookahead = 0.0;

    for (size_t lookahead_idx = 0;
         lookahead_idx < lookahead_ladder.size() && !selected_path.has_value();
         ++lookahead_idx) {
        const auto lookahead = lookahead_ladder[lookahead_idx];
        const auto goal_local = find_local_goal(transform, lookahead);
        if (goal_local.has_value()) {
            last_attempt_goal = goal_local.value();
            const auto result = core_->plan(goal_local.value(), scan_frame, manual_obstacles_local);
            if (result.status == PlanStatus::OK && result.path.has_value()) {
                selected_goal = goal_local.value();
                selected_path = std::move(result.path.value());
                selected_lookahead = lookahead;
            }
        }
    }

    if (!selected_path.has_value()) {
        if (!last_attempt_goal.has_value()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "No valid local goal found from global path");
            publish_debug_markers("failed: no goal", std::nullopt, std::nullopt, nullptr);
        } else {
            const auto &goal = last_attempt_goal.value();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "No collision-free path found for goal ladder. Last attempt x=%.2f y=%.2f z=%.2f scan=%s ranges=%zu",
                goal.x,
                goal.y,
                goal.z,
                (use_scan_obstacles_ && have_scan_) ? "yes" : "no",
                (use_scan_obstacles_ && have_scan_) ? scan_.ranges.size() : 0);
            publish_debug_markers("failed", last_attempt_goal, std::nullopt, &core_->last_debug());
        }
        publish_ready(false);
        return;
    }

    nav_msgs::msg::Path path_msg = to_path_msg(selected_path.value());

    const auto mode_base = core_->last_debug().used_straight ? std::string("straight") : std::string("rrt");
    std::stringstream mode;
    mode << mode_base << " L=" << selected_lookahead;
    local_path_pub_->publish(path_msg);
    debug_active_path_pub_->publish(path_msg);
    publish_debug_markers(mode.str(), selected_goal, path_msg, &core_->last_debug());
    publish_ready(true);
}

size_t LocalPlannerNode::find_nearest_index() const {
    size_t nearest_idx = 0;
    double nearest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < global_path_.poses.size(); ++i) {
        const auto &point = global_path_.poses[i].pose.position;
        const double dist = std::hypot(
            point.x - odom_.pose.pose.position.x,
            point.y - odom_.pose.pose.position.y);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

std::optional<geometry_msgs::msg::Point> LocalPlannerNode::find_local_goal(
    const geometry_msgs::msg::TransformStamped &transform,
    double lookahead_distance) const {
    if (global_path_.poses.empty()) {
        return std::nullopt;
    }

    const size_t nearest_idx = find_nearest_index();
    const size_t n = global_path_.poses.size();
    double accumulated = 0.0;

    std::optional<geometry_msgs::msg::Point> reachable_fallback;

    bool within_planning_window = true;
    for (size_t step = 0; step < n && within_planning_window; ++step) {
        const size_t curr_idx = (nearest_idx + step) % n;
        if (step > 0) {
            const size_t prev_idx = (nearest_idx + step - 1) % n;
            const auto &prev = global_path_.poses[prev_idx].pose.position;
            const auto &curr = global_path_.poses[curr_idx].pose.position;
            accumulated += std::hypot(curr.x - prev.x, curr.y - prev.y);
            within_planning_window = accumulated <= core_params_.planning_distance;
        }

        if (within_planning_window) {
            const auto local = transform_point(global_path_.poses[curr_idx].pose.position, transform);
            const bool inside_forward_box =
                local.x >= core_params_.step_size &&
                local.x <= core_params_.planning_distance &&
                std::abs(local.y) <= core_params_.max_lateral;

            if (inside_forward_box) {
                reachable_fallback = local;
                if (accumulated >= lookahead_distance) {
                    return local;
                }
            }
        }
    }

    if (reachable_fallback.has_value()) {
        return reachable_fallback;
    }

    return std::nullopt;
}

geometry_msgs::msg::Point LocalPlannerNode::transform_point(
    const geometry_msgs::msg::Point &point,
    const geometry_msgs::msg::TransformStamped &transform) const {
    geometry_msgs::msg::PointStamped source;
    source.header.frame_id = global_frame_id_;
    source.header.stamp = transform.header.stamp;
    source.point.x = point.x;
    source.point.y = point.y;
    source.point.z = 0.0;

    geometry_msgs::msg::PointStamped transformed;
    tf2::doTransform(source, transformed, transform);

    geometry_msgs::msg::Point result = transformed.point;
    result.z = point.z;
    return result;
}

nav_msgs::msg::Path LocalPlannerNode::to_path_msg(const LocalPath &local_path) {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = core_params_.local_frame_id;
    msg.header.stamp = this->now();
    msg.poses = local_path.poses;
    for (auto &pose : msg.poses) {
        pose.header.stamp = msg.header.stamp;
    }
    return msg;
}

std::vector<geometry_msgs::msg::Point> LocalPlannerNode::manual_obstacles_to_local(
    const geometry_msgs::msg::TransformStamped &global_to_local) {
    const auto now_time = this->now();
    if (manual_obstacle_lifetime_ > 0.0) {
        const auto max_age = rclcpp::Duration::from_seconds(manual_obstacle_lifetime_);
        manual_obstacles_.erase(
            std::remove_if(
                manual_obstacles_.begin(),
                manual_obstacles_.end(),
                [&](const ManualObstacle &obstacle) {
                    return (now_time - obstacle.created_at) > max_age;
                }),
            manual_obstacles_.end());
    }

    std::vector<geometry_msgs::msg::Point> local_obstacles;
    local_obstacles.reserve(manual_obstacles_.size());
    for (const auto &obstacle : manual_obstacles_) {
        auto local = transform_point(obstacle.point_map, global_to_local);
        local.z = 0.0;
        if (local.x >= core_params_.scan_front_min_x &&
            local.x < core_params_.planning_distance &&
            std::abs(local.y) <= core_params_.max_lateral) {
            local_obstacles.push_back(local);
        }
    }

    return local_obstacles;
}

void LocalPlannerNode::clicked_point_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (max_manual_obstacles_ == 0U) {
        return;
    }

    geometry_msgs::msg::PointStamped point_map;
    const std::string source_frame = msg->header.frame_id.empty() ? global_frame_id_ : msg->header.frame_id;
    if (source_frame == global_frame_id_) {
        point_map = *msg;
        point_map.header.frame_id = global_frame_id_;
    } else {
        geometry_msgs::msg::PointStamped source = *msg;
        source.header.frame_id = source_frame;
        try {
            const auto source_to_map = tf_buffer_->lookupTransform(
                global_frame_id_, source_frame, tf2::TimePointZero);
            tf2::doTransform(source, point_map, source_to_map);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(
                this->get_logger(),
                "Ignoring clicked obstacle in frame %s: cannot transform to %s: %s",
                source_frame.c_str(),
                global_frame_id_.c_str(),
                ex.what());
            return;
        }
    }

    point_map.point.z = 0.0;
    manual_obstacles_.push_back({point_map.point, this->now()});
    if (manual_obstacles_.size() > max_manual_obstacles_) {
        manual_obstacles_.erase(manual_obstacles_.begin());
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Added temporary obstacle at %s x=%.2f y=%.2f (%zu active)",
        global_frame_id_.c_str(),
        point_map.point.x,
        point_map.point.y,
        manual_obstacles_.size());
    publish_debug_markers("clicked obstacle", std::nullopt, std::nullopt, nullptr);
}

void LocalPlannerNode::clear_manual_obstacles_callback(
    const std_msgs::msg::Empty::SharedPtr /*msg*/) {
    manual_obstacles_.clear();
    publish_debug_markers("manual obstacles cleared", std::nullopt, std::nullopt, nullptr);
    RCLCPP_INFO(this->get_logger(), "Cleared temporary planner obstacles");
}

void LocalPlannerNode::publish_debug_markers(
    const std::string &mode,
    const std::optional<geometry_msgs::msg::Point> &goal_local,
    const std::optional<nav_msgs::msg::Path> &active_path,
    const PlanDebug *debug) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    auto make_marker = [&](int id, int type, const std::string &ns, const std::string &frame) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        return marker;
    };

    int id = 1;
    auto text_marker = make_marker(id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, "mode", core_params_.local_frame_id);
    text_marker.pose.position.x = 0.5;
    text_marker.pose.position.z = 0.7;
    text_marker.scale.z = 0.25;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = "local planner: " + mode;
    markers.markers.push_back(text_marker);

    if (goal_local.has_value()) {
        auto goal = make_marker(id++, visualization_msgs::msg::Marker::SPHERE, "selected_goal", core_params_.local_frame_id);
        goal.pose.position = goal_local.value();
        goal.scale.x = 0.18;
        goal.scale.y = 0.18;
        goal.scale.z = 0.18;
        goal.color.r = 1.0;
        goal.color.g = 0.85;
        goal.color.b = 0.05;
        markers.markers.push_back(goal);
    }

    if (active_path.has_value() && !active_path->poses.empty()) {
        auto line = make_marker(id++, visualization_msgs::msg::Marker::LINE_STRIP, "active_path", active_path->header.frame_id);
        line.scale.x = 0.07;
        line.color.g = 1.0;
        for (const auto &pose : active_path->poses) {
            line.points.push_back(pose.pose.position);
        }
        markers.markers.push_back(line);
    }

    if (!manual_obstacles_.empty()) {
        auto manual = make_marker(
            id++,
            visualization_msgs::msg::Marker::SPHERE_LIST,
            "manual_obstacles_map",
            global_frame_id_);
        const double diameter = std::max(0.12, core_params_.obstacle_inflation * 2.0);
        manual.scale.x = diameter;
        manual.scale.y = diameter;
        manual.scale.z = 0.12;
        manual.color.r = 1.0;
        manual.color.g = 0.35;
        manual.color.b = 0.0;
        for (const auto &obstacle : manual_obstacles_) {
            manual.points.push_back(obstacle.point_map);
        }
        markers.markers.push_back(manual);
    }

    if (debug != nullptr) {
        if (!debug->obstacles.empty()) {
            auto obs = make_marker(id++, visualization_msgs::msg::Marker::POINTS, "scan_obstacles", core_params_.local_frame_id);
            obs.scale.x = 0.08;
            obs.scale.y = 0.08;
            obs.color.r = 1.0;
            obs.color.g = 0.05;
            obs.color.b = 0.05;
            obs.points = debug->obstacles;
            markers.markers.push_back(obs);
        }

        if (debug->rrt.nodes.size() > 1) {
            auto tree = make_marker(id++, visualization_msgs::msg::Marker::LINE_LIST, "rrt_tree", core_params_.local_frame_id);
            tree.scale.x = 0.015;
            tree.color.r = 0.1;
            tree.color.g = 0.65;
            tree.color.b = 1.0;
            tree.color.a = 0.45;
            for (size_t i = 1; i < debug->rrt.nodes.size(); ++i) {
                const auto &node = debug->rrt.nodes[i];
                const bool parent_valid =
                    node.parent >= 0 && static_cast<size_t>(node.parent) < debug->rrt.nodes.size();
                if (parent_valid) {
                    geometry_msgs::msg::Point a;
                    a.x = node.x;
                    a.y = node.y;
                    geometry_msgs::msg::Point b;
                    b.x = debug->rrt.nodes[node.parent].x;
                    b.y = debug->rrt.nodes[node.parent].y;
                    tree.points.push_back(a);
                    tree.points.push_back(b);
                }
            }
            markers.markers.push_back(tree);
        }

        if (!debug->rrt.raw_path.empty()) {
            auto raw = make_marker(id++, visualization_msgs::msg::Marker::LINE_STRIP, "rrt_selected_raw", core_params_.local_frame_id);
            raw.scale.x = 0.05;
            raw.color.r = 1.0;
            raw.color.b = 1.0;
            raw.points = debug->rrt.raw_path;
            markers.markers.push_back(raw);
        }
    }

    debug_markers_pub_->publish(markers);
}

void LocalPlannerNode::load_track_map() {
    if (debug_map_loaded_ || !debug_map_enabled_) {
        return;
    }
    debug_map_loaded_ = true;

    std::string file_path;
    try {
        file_path = ament_index_cpp::get_package_share_directory(debug_map_package_) + debug_map_file_;
    } catch (const std::exception &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not find debug map package %s: %s", debug_map_package_.c_str(), ex.what());
        return;
    }

    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open debug map csv: %s", file_path.c_str());
        return;
    }

    struct CsvPoint {
        double x;
        double y;
        double right;
        double left;
    };
    std::vector<CsvPoint> rows;
    std::string line;
    while (std::getline(file, line)) {
        const bool parseable_line = !line.empty() && line[0] != '#';
        if (parseable_line) {
            std::stringstream ss(line);
            std::string x_str, y_str, right_str, left_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, right_str, ',');
            std::getline(ss, left_str, ',');
            try {
                rows.push_back({std::stod(x_str), std::stod(y_str), std::stod(right_str), std::stod(left_str)});
            } catch (const std::exception &) {
            }
        }
    }

    if (rows.size() < 2) {
        return;
    }

    track_map_.clear();
    track_map_.reserve(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto &prev = rows[(i + rows.size() - 1) % rows.size()];
        const auto &next = rows[(i + 1) % rows.size()];
        double dx = next.x - prev.x;
        double dy = next.y - prev.y;
        const double len = std::hypot(dx, dy);
        if (len <= 1e-9) {
            dx = 1.0;
            dy = 0.0;
        } else {
            dx /= len;
            dy /= len;
        }
        const double nx = -dy;
        const double ny = dx;

        TrackMapPoint point;
        point.center.x = rows[i].x;
        point.center.y = rows[i].y;
        point.left_boundary.x = rows[i].x + nx * rows[i].left;
        point.left_boundary.y = rows[i].y + ny * rows[i].left;
        point.right_boundary.x = rows[i].x - nx * rows[i].right;
        point.right_boundary.y = rows[i].y - ny * rows[i].right;
        track_map_.push_back(point);
    }
}

void LocalPlannerNode::publish_track_map() {
    if (debug_map_published_ || !debug_map_enabled_) {
        return;
    }
    load_track_map();
    if (track_map_.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    auto make_line = [&](int id, const std::string &ns, double r, double g, double b, double width) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_id_;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = width;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        return marker;
    };

    auto center = make_line(1, "centerline", 0.2, 0.9, 0.2, 0.04);
    auto left = make_line(2, "left_boundary", 0.9, 0.9, 0.9, 0.035);
    auto right = make_line(3, "right_boundary", 0.9, 0.9, 0.9, 0.035);
    for (const auto &point : track_map_) {
        center.points.push_back(point.center);
        left.points.push_back(point.left_boundary);
        right.points.push_back(point.right_boundary);
    }
    center.points.push_back(track_map_.front().center);
    left.points.push_back(track_map_.front().left_boundary);
    right.points.push_back(track_map_.front().right_boundary);
    markers.markers.push_back(center);
    markers.markers.push_back(left);
    markers.markers.push_back(right);

    auto ribs = make_line(4, "track_width_samples", 0.35, 0.35, 0.35, 0.01);
    ribs.type = visualization_msgs::msg::Marker::LINE_LIST;
    ribs.color.a = 0.45;
    const size_t stride = std::max<size_t>(1, track_map_.size() / 120);
    for (size_t i = 0; i < track_map_.size(); i += stride) {
        ribs.points.push_back(track_map_[i].left_boundary);
        ribs.points.push_back(track_map_[i].right_boundary);
    }
    markers.markers.push_back(ribs);

    debug_map_pub_->publish(markers);
    debug_map_published_ = true;
}

void LocalPlannerNode::publish_ready(bool ready) {
    std_msgs::msg::Bool msg;
    msg.data = ready;
    ready_pub_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
