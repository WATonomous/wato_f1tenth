#include "keyboard_teleop.hpp"

KeyboardTeleop::KeyboardTeleop () : Node ("keyboard_teleop_node") {
    //parameters
    this->declare_parameter("polling_frequency", 60);
    this->declare_parameter("key_speed", 1.0);
    this->declare_parameter("key_angle", M_PI/2);
    this->declare_parameter("output_topic_name", "/ackermann_cmd");

    poll_frequency = this->get_parameter("polling_frequency").as_int();
    Key_speed = this->get_parameter("key_speed").as_double();
    key_angle = this->get_parameter("key_angle").as_double();
    output_topic = this->get_parameter("output_topic_name").as_string();

    //publisher
    drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(output_topic, 10);

    //timer
    timer_keyinput = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/poll_frequency)),
        std::bind(&KeyboardTeleop::pollKeyInput,this));

    //setting defult 
    current_drive.header.frame_id = "base_link";
    current_drive.header.stamp = this->now();
    current_drive.drive.speed = 0.0;
    current_drive.drive.steering_angle = 0.0;

    KeyboardTeleop::setRawMode(true);
    KeyboardTeleop::printInstruction();

}

KeyboardTeleop::~KeyboardTeleop() {
    setRawMode(false);

    current_drive.header.stamp = this->now();
    current_drive.drive.speed = 0.0;
    current_drive.drive.steering_angle = 0.0;

    KeyboardTeleop::drive_pub->publish(current_drive);

    RCLCPP_INFO(this->get_logger(), "keyboard teleop has stopped");
}

void KeyboardTeleop::setRawMode(bool enable) {

    if (enable) {

        // Save old settings
        tcgetattr(STDIN_FILENO, &old_terminal_settings_);

        // Set terminal to raw mode
        struct termios raw = old_terminal_settings_;
        raw.c_lflag &= ~(ECHO | ICANON); // Disable echo and canonical mode
        raw.c_cc[VMIN] = 0;              // Non-blocking read
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        // Set stdin to non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    } else {

        // Restore old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);

    }

}

void KeyboardTeleop::pollKeyInput() {
    char c;
    bool key_pressed = false;

    if (read(STDIN_FILENO, &c, 1) == 1) { // keypress dected 

        key_pressed = true;
        KeyboardTeleop::processInput(c);

    } else { // no keypress dected

        current_drive.drive.speed = 0.0;
        current_drive.drive.steering_angle = 0.0;

    }

    current_drive.header.stamp = this->now();
    drive_pub->publish(current_drive);
}

void KeyboardTeleop::processInput(char key) {

    switch (key) {
        case 'w':
            current_drive.drive.speed = Key_speed;
            break;

        case 'a':
            current_drive.drive.steering_angle = key_angle;
            break;

        case 's':
            current_drive.drive.speed = -Key_speed;
            break;

        case 'd':
            current_drive.drive.steering_angle = -key_angle;
            break;

        case '-':
            Key_speed -= 0.5;
            if (Key_speed < -5.0) {
                Key_speed = -5.0;
            }
            break;

        case '+':
            Key_speed += 0.5;
            if (Key_speed > 8.0) {
                Key_speed = 8.0;
            }
            break;

        case '\x03':
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
            break;

        default:
            break;
    }

}

void KeyboardTeleop::printInstruction() {
    RCLCPP_INFO(this->get_logger(),"\n"
        "keyboard controls:"
        "w - forward \n"
        "a - left \n"
        "s - back \n"
        "d - right \n"
        "+ - speed up \n"
        "- - speed down \n"
        "current speed : %f \n"
        "current steering :%f \n",
        static_cast<float>(current_drive.drive.speed),
        static_cast<float>(current_drive.drive.steering_angle)
    );
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}