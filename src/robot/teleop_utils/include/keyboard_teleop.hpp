#ifndef KEYBOARD_TELEOP
#define KEYBOARD_TELEOP

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class KeyboardTeleop : public rclcpp::Node {
public : 

    KeyboardTeleop();
    ~KeyboardTeleop();

private:

   //pubs and subs  
   rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

   //timer
   rclcpp::TimerBase::SharedPtr timer_keyinput;

   //terminos for input handling
   struct termios old_terminal_settings_;

   //data
   ackermann_msgs::msg::AckermannDriveStamped current_drive;

   //helper functions
   void setRawMode(bool enable);
   void pollKeyInput();
   void processInput(char c);
   void printInstruction();

   //parameters
   std::string output_topic;
   int poll_frequency;
   double Key_speed, key_angle;

};


#endif