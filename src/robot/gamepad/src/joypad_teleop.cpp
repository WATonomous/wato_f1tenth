#include "joypad_teleop.hpp"

/*
future todo 
give it gears for fun and a button to turn off the gear
gears as in ability to shift though gears like a gt car
*/

JOYPAD::JOYPAD () : Node ("joypad_node") {

    //parameters
    this->declare_parameter<std::string>("gamepad_topic","/drive/joystick");
    this->declare_parameter<std::string>("joy_topic","/joy");
    this->declare_parameter<std::string>("my_frame_id","base_link");

    this->declare_parameter<float>("max_speed",22.88);
    this->declare_parameter<float>("steering_gain",0.5236);
    this->declare_parameter<float>("max_steering_rate",3.2);

    gamepad_topic = this->get_parameter("gamepad_topic").as_string();
    joy_topic = this->get_parameter("joy_topic").as_string();
    my_frame_id = this->get_parameter("my_frame_id").as_string();

    max_speed = this->get_parameter("max_speed").as_double();
    max_steering_rate = this->get_parameter("max_steering_rate").as_double();
    steering_gain = this->get_parameter("steering_gain").as_double();


    //pubs and sub
    ackerman_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        gamepad_topic, 10);

    gamepad_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic,20 , std::bind(&JOYPAD::gamepadCallback, this, std::placeholders::_1));

    //emergancy drive message initalization
    emergancy_drive_msg.speed = 0.0;
    emergancy_drive_msg.steering_angle = 0.0;

}

void JOYPAD::gamepadCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    ackermann_msgs::msg::AckermannDrive drive;
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;

    drive_msg.header.frame_id = my_frame_id;
    drive_msg.header.stamp = msg->header.stamp;

    bool pit_speed_limit = false;

    //emergancy / deadman switch
    if (!msg->buttons.at(0)) {
        drive_msg.drive = emergancy_drive_msg;
        ackerman_pub->publish(drive_msg);
        //RCLCPP_INFO(this->get_logger(),"press x to activate drive");
        return;
    }

    float current_steering = JOYPAD::steering_mapper(msg->axes.at(0));
    float current_throtel = JOYPAD::trigger_maper(msg->axes.at(5));
    float current_break = JOYPAD::trigger_maper(msg->axes.at(2));

    //reversing
    if (msg->buttons.at(1)) {
        drive.speed = -0.25;
        drive.steering_angle = current_steering;
        drive_msg.drive = drive;
        ackerman_pub->publish(drive_msg);
        //RCLCPP_INFO(this->get_logger(),"reverse is active");
        return;
    }

    if (msg->buttons.at(2)) {
        pit_speed_limit = true;
    }

    //throtel logic
    float total_throtel = current_throtel - current_break;
    if (total_throtel < 0.0 ) {
        total_throtel = 0.0;
    }
   

    drive.speed = total_throtel * max_speed;
    drive.steering_angle = current_steering;
    drive.steering_angle_velocity = max_steering_rate;

    if (pit_speed_limit && drive.speed > 1.0) {
        drive.speed = 1.0;
    }

    //publish the message
    drive_msg.drive = drive;
    ackerman_pub->publish(drive_msg);
}

float JOYPAD::trigger_maper(const float r2) {
    float ct = -(r2 - 1) / 2;
    return (std::exp(4 * (ct - 1)) - 0.01832);
}

float JOYPAD::steering_mapper(const float ls) {
    return ls * steering_gain;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JOYPAD>());
  rclcpp::shutdown();
  return 0;
}