#include "global_planner.hpp"

GlobalPlanner::GlobalPlanner () : Node ("global_planner_node") {

    //parameters
    this->declare_parameter<std::string>("file_directory", "/assets/autoDriveRaceline_with_vel.csv");
    this->declare_parameter<std::string>("vis_topic", "/global_planner/vis");
    this->declare_parameter<std::string>("path_topic", "/global_planner/path");
    this->declare_parameter<std::string>("waypoint_frame_id","map");

    //init pub and subs
    file_directory = this->get_parameter("file_directory").as_string();
    path_pub_topic = this->get_parameter("path_topic").as_string();
    vis_pub_topic = this->get_parameter("vis_topic").as_string();
    waypoint_frame_id = this->get_parameter("waypoint_frame_id").as_string();


    //declare pubs and subs
    auto qos = rclcpp::QoS(1).transient_local().reliable();

    path_pub = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic,qos);

    //vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(vis_pub_topic,10);

    //check if the file is valid
    const auto file_path = ament_index_cpp::get_package_share_directory("global_planner") + file_directory;
    std::ifstream file (file_path);
    if (!file.is_open()) {
        RCLCPP_FATAL(this->get_logger(), "could not open the file, check file path again");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "the csv file %s opended correctly", file_path.c_str());

    //init some of the data
    waypoints.header.frame_id = waypoint_frame_id;
    waypoints.header.stamp = this->now();

    //populate the correct data
    GlobalPlanner::retrieve_data(file);

    RCLCPP_INFO(this->get_logger(),"number of waypoints : %zu \n", waypoints.poses.size());

    //publish the data
    GlobalPlanner::publish_data();


}

void GlobalPlanner::retrieve_data(std::ifstream &file) {

    std::string line;

    //skip the header
    std::getline(file, line);

    while (std::getline(file, line)) {

       //convert line to string stream
       std::stringstream ss(line);
       std::string x_str, y_str, vel_str;
       std::string temp;

       //break string stream into parts
       std::getline(ss, temp, ',');
       std::getline(ss, x_str, ',');
       std::getline(ss, y_str, ',');
       std::getline(ss, temp, ',');
       std::getline(ss, temp, ',');
       std::getline(ss, vel_str, ',');

       //convert to double
       double x,y,vel;
       try {
            x = std::stod(x_str);
            y = std::stod(y_str);
            vel = std::stod(vel_str);
       } catch (const std::invalid_argument &e){
            RCLCPP_WARN(get_logger(), "Skipping invalid line: %s", line.c_str());
            continue;
       }

       //package the waypoint and velocity
       geometry_msgs::msg::PoseStamped current_waypoint;
       current_waypoint.header.frame_id = waypoint_frame_id;
       current_waypoint.header.stamp = this->now();

       //might have to play with these to fix the wierd ofset
       current_waypoint.pose.position.x = x;
       current_waypoint.pose.position.y = y + 15.5;

       //contains the velocity
       current_waypoint.pose.position.z = vel;

       waypoints.poses.push_back(current_waypoint);

    }

    //close the file
    file.close();
}

void GlobalPlanner::publish_data () {
    path_pub->publish(waypoints);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}