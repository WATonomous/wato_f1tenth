#include "wheel_odom.hpp"

WheelOdom::WheelOdom () : Node ("Wheel_Odom_Node") {
    //initalize the subs

    right_encoder_sub = this->create_subscription<sensor_msgs::msg::JointState> (
        "/autodrive/f1tenth_1/right_encoder",10,[this](const sensor_msgs::msg::JointState::SharedPtr msgs){right_encoder_data = msgs;});

    left_encoder_sub = this->create_subscription<sensor_msgs::msg::JointState> (
        "/autodrive/f1tenth_1/left_encoder",10,[this](const sensor_msgs::msg::JointState::SharedPtr msgs){left_encoder_data = msgs;});

    steering_sub = this->create_subscription<std_msgs::msg::Float32>(
        "/autodrive/f1tenth_1/steering",10,[this](const std_msgs::msg::Float32::SharedPtr msgs){steering_data = msgs;});

    //initalize the pubs

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom",10);

    //initalize the timer (50 ms is the standard update interval of for odometrey)

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&WheelOdom::broadcastTransform,this));

    //initalize the tf broadcaster

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    /*
        initalize base vals for robot's starting location, 
        based off of how the robot is oriented at the start of 
        the simulation relative to the world frame
        (determined using the ips and intutions)
    */
    x = 0.7412, y = 3.1583, yaw = -M_PI/2,left_encoder_last = 0,right_encoder_last = 0,right_encoder_current = 0,left_encoder_current = 0;

    //initalize the child and parent frame (change up the name of the child frame later)
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    od.header.frame_id = "odom";
    od.child_frame_id = "base_link";
    od.twist.twist.linear.y = 0;

    od.pose.covariance[0] = 0.15;  // x
    od.pose.covariance[7] = 0.35;  // y
    od.pose.covariance[14] = 20; // z (2D robot, so no z info)
    od.pose.covariance[21] = 22;
    od.pose.covariance[28] = 33;
    od.pose.covariance[35] = 0.30;  // yaw

    od.twist.covariance[0] = 0.05;    // vx: linear x (trusted)
    od.twist.covariance[7] = 55;     // vy: linear y (not measured)
    od.twist.covariance[14] = 13;    // vz: linear z (not measured)

    od.twist.covariance[21] = 22;    // angular x (not measured)
    od.twist.covariance[28] = 22;    // angular y (not measured)
    od.twist.covariance[35] = 0.33;    // angular z (trusted)  

    RCLCPP_INFO(this->get_logger(),"initalized constructor");

}

/**
 * @brief
 * the function uses the most recent data from the wheel encoders
 * to update the robots pose : x,y,yaw,linear x velocity and 
 * angular velocity. the update scheme is carried out using a 'tricycle' 
 * model. the function uses mid-point integration for a more numericaly 
 * stable outcome
 * 
 * @return
 * the robot's updated pose 
 */
void WheelOdom::calculateOdom() {

   // do a null ptr check before going for calculaitons
    if (!steering_data || !right_encoder_data || !left_encoder_data) {
        RCLCPP_WARN(this->get_logger(), "incomplete data, can't update state");
        return;
    }

    //check if data is available 
    if (right_encoder_data->position.empty() || left_encoder_data->position.empty()) {
        RCLCPP_INFO(this->get_logger(),"there is no valid data from encoders");
        return;
    } 

    //get curret steering angle
    double current_steering = steering_data->data;

    //estimate current linear velocity
    right_encoder_current = right_encoder_data->position[0];
    left_encoder_current = left_encoder_data->position[0];

    double right_delta = right_encoder_current - right_encoder_last;
    double left_delta = left_encoder_current - left_encoder_last;

    double left_distance = right_delta * WHEEL_RADIUS;
    double right_distance = left_delta * WHEEL_RADIUS;

    // double left_distance = (left_delta/360) * (M_PI * WHEEL_RADIUS * 2);
    // double right_distance = (right_delta/360) * (M_PI * WHEEL_RADIUS * 2);

    double r_velocity = right_distance/DT;
    double l_velocity = left_distance/DT;

    double velocity = (r_velocity + l_velocity) / 2;

    double angular_velocity = 0.0;

    
    if (std::abs(current_steering) < STEERING_THRESHOLD) {
        //going straight case
        x += velocity * std::cos(yaw) * DT;
        y += velocity * std::sin(yaw) * DT;
    } else {
        //calculate current turning radius
        double turn_radius = WHEELBASE / std::tan(current_steering);

        //current angular velocity
        angular_velocity = velocity / turn_radius;

        //diffrence in yaw from last time step
        double delta_yaw = angular_velocity * DT;

        double mid_yaw = yaw + delta_yaw/2.0;

        //update the the current state (mid point method)
        x += turn_radius * (sin(mid_yaw + delta_yaw) - sin(mid_yaw));
        y += -turn_radius * (cos(mid_yaw + delta_yaw) - cos(mid_yaw));
        yaw += delta_yaw;
    }

    //normalize yaw
    yaw = std::atan2(std::sin(yaw),std::cos(yaw));

    //update curret linear and angular velocity
    od.twist.twist.linear.x = velocity;
    od.twist.twist.angular.z = angular_velocity;

    //store previous encoder state
    right_encoder_last = right_encoder_current;
    left_encoder_last = left_encoder_current;   

}

/**
 * @brief 
 * the main call back function what works on a fixed timer of 20 ms. 
 * when the call back occures, the functions call the calculateOdom
 * to get the most recent pose. than the function publishes that pose 
 * 
 * @return 
 * pubish the odometrey.
 */
void WheelOdom::broadcastTransform() {

    if (!should_update) {
        if (left_encoder_data && right_encoder_data) {
            right_encoder_last = right_encoder_data->position[0];
            left_encoder_last = left_encoder_data->position[0];
            should_update = true;
            RCLCPP_INFO(this->get_logger(),"initalized encoders");
        }
        return;
    }

    //calculate the current odom
    WheelOdom::calculateOdom();

    //set the clock
    od.header.stamp = this ->get_clock()->now();

    //set the odom position
    od.pose.pose.position.x = x;
    od.pose.pose.position.y = y;
    od.pose.pose.position.z = Z_VAL;

    //set the yaw angle
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);

    //set odom orientation
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();

    //publish
    odom_pub->publish(od);

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdom>());
  rclcpp::shutdown();
  return 0;
}
