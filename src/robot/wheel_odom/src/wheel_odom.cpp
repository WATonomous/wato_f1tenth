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

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odometry",10);

    //initalize the timer (50 ms is the standard update interval of for odometrey)

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&WheelOdom::broadcastTransform,this));

    //initalize the tf broadcaster

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //initalize base vals
    x = 0, y = 0, yaw = 0,left_encoder_last = 0,right_encoder_last = 0,right_encoder_curret = 0,left_encoder_current = 0;

    //initalize the child and parent frame (change up the name of the child frame later)
    t.header.frame_id = "odom";
    t.child_frame_id = "f1tenth_1";

    od.header.frame_id = "odom";
    od.child_frame_id = "f1tenth_1";

    RCLCPP_INFO(this->get_logger(),"initalized constructor");

}

//calculate odometrey based of the bycycle model 
//(shoulud be good enough before filtering with ekf or slam)
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
    RCLCPP_INFO(this->get_logger(),"curret steering angle in radian = %f", current_steering);
    
    //clap the steering angle
    if (current_steering > STEERING_NORMAL) current_steering = STEERING_NORMAL;
    if (current_steering < -STEERING_NORMAL) current_steering = -STEERING_NORMAL;

    //estimate current linear velocity
    right_encoder_curret = right_encoder_data->position[0];
    left_encoder_current = left_encoder_data->position[0];

    double right_delta = right_encoder_curret - right_encoder_last;
    double left_delta = left_encoder_current - left_encoder_last;

    double left_distance = (2 * M_PI * WHEEL_RADIUS * left_delta)/ TICKS_PER_REVELOUTION;
    double right_distance = (2 * M_PI * WHEEL_RADIUS * right_delta)/ TICKS_PER_REVELOUTION;

    double r_velocity = right_distance/DT;
    double l_velocity = left_distance/DT;

    double velocity = (r_velocity + l_velocity) / 2;

    double angular_velocity = 0.0;

    
    if (std::abs(current_steering) < 1e-4) {
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

        //update the the current state
        x += turn_radius * (std::sin (delta_yaw + yaw) - std::sin (yaw));
        y += -turn_radius * (std::cos (delta_yaw+ yaw) - std::cos (yaw));
        yaw += delta_yaw;
    }

    //normalize yaw
    yaw = std::atan2(std::sin(yaw),std::cos(yaw));

    //update curret linear and angular velocity
    od.twist.twist.linear.x = velocity;
    od.twist.twist.angular.z = angular_velocity;

    //store previous encoder state
    right_encoder_last = right_encoder_curret;
    left_encoder_last = left_encoder_current;   

    RCLCPP_INFO(this->get_logger(),"x = %f, y = %f, yaw = %f, v = %f , av = %f", x , y, yaw, velocity, angular_velocity);

}

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
    t.header.stamp = this->get_clock()->now();
    od.header.stamp = this ->get_clock()->now();

    //set the translations
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    //set the odom position
    od.pose.pose.position.x = x;
    od.pose.pose.position.y = y;
    od.pose.pose.position.z = 0.0;

    //set the yaw angle
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    //set odom orientation
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();

    //broadcast the transform
    if (broadcast_transform)
        tf_broadcaster->sendTransform(t);

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
