#include "global_planner.hpp"

GlobalPlanner::GlobalPlanner () : Node ("global_planner_node") {
    //parameters

    this->declare_parameter<std::string>("file_path", "/assets/autoDriveRaceline_with_vel.cs");
    this->declare_parameter<std::string>("velocity_topic", "/global_planner/vel");
    this->declare_parameter<std::string>("vis_topic", "/global_planner/vis");
    this->declare_parameter<std::string>("path_topic", "/global_planner/path");

    //init pub and subs
    file_path = this->get_parameter("file_path").as_string();
    velocity_pub_topic = this->get_parameter("velocity_topic").as_string();
    path_pub_topic = this->get_parameter("path_topic").as_string();
    vis_pub_topic = this->get_parameter("vis_topic").as_string();

    //declare pubs and subs
    velocity_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(velocity_pub_topic,10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic,10);
    vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(vis_pub_topic,10);

    //check if the file is valid
    std::ifstream file (file_path);
    if (!file.is_open()) {
        RCLCPP_FATAL(this->get_logger(), "could not open the file, check file path again");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "the csv file %s opended correctly", file_path.c_str());

    //populate the correct data


}

void GlobalPlanner::retrive_data(std::ifstream &file) {

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

       //package the data
       

    }

    //close the file
    file.close();
}

void GlobalPlanner::publish_data () {

}