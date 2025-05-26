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

    //initalize the child and parent frame
    t.header.frame_id = "odom";
    t.child_frame_id = "f1tenth_1";

    od.header.frame_id = "odom";
    od.child_frame_id = "f1tenth_1";

    RCLCPP_INFO(this->get_logger(),"initalized constructor");

}

//calculate odometrey based of the bycycle model 
//(shoulud be good enough before filtering with ekf or slam)
void WheelOdom::calculateOdom() {

    //calculate the curret steering angle
    if (!steering_data) {
        RCLCPP_WARN(this->get_logger(), "steering data not yet recived");
        return;
    }

    double current_steering = STEERING_NORMAL * steering_data->data;
    
    if (current_steering > STEERING_NORMAL) 
        current_steering = STEERING_NORMAL;

    if (current_steering < -STEERING_NORMAL)
        current_steering = -STEERING_NORMAL;

        
    //check for null ptr
    if (!right_encoder_data || !left_encoder_data) {
        RCLCPP_WARN(this->get_logger(), "Encoder data not yet received.");
        return;
    }

    //check data validity
    if (right_encoder_data->position.empty() || left_encoder_data->position.empty()) {
        should_update = false;
        RCLCPP_INFO(this->get_logger(),"there is no valid data from encoders");
    } else {
        should_update = true;
    }

    if (!should_update)
        return;

    //RCLCPP_INFO(this->get_logger(),"there is valid data");
    //estimate current linear velocity
    right_encoder_curret = right_encoder_data->position[0];
    left_encoder_current = left_encoder_data->position[0];

    double right_delta = right_encoder_curret - right_encoder_last;
    double left_delta = left_encoder_current - left_encoder_last;

    right_encoder_last = right_encoder_curret;
    left_encoder_last = left_encoder_current;

    double left_distance = (2 * M_PI * WHEEL_RADIUS * left_delta)/TICKS_PER_REVELOUTION;
    double right_distance = (2 * M_PI * WHEEL_RADIUS * right_delta)/TICKS_PER_REVELOUTION;

    double r_velocity = right_distance/DT;
    double l_velocity = left_distance/DT;

    double velocity = (r_velocity + l_velocity) / 2;

    //compute heading change

    double angular_velocity = (velocity/WHEELBASE) * std::tan(current_steering);
    double delta_yaw = angular_velocity * DT;

    RCLCPP_INFO(this->get_logger(),"curret velocity is %f",velocity);

    // if (std::isnan(angular_velocity) || std::isnan(delta_yaw))
    //     RCLCPP_INFO(this->get_logger(),"angular velocity or delta yaw is a nan");

    //update current pose
    x += velocity * std::cos (yaw) * DT;
    y += velocity * std::sin (yaw) * DT;
    yaw += delta_yaw; 
    

    //normalive yaw
    if (yaw < -M_PI || yaw > M_PI)
        yaw = std::atan2(std::sin(yaw),std::cos(yaw));

    od.twist.twist.linear.x = velocity;

    RCLCPP_INFO(this->get_logger(),"x = %f, y = %f, yaw = %f",x ,y, yaw);

}

void WheelOdom::broadcastTransform() {

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
