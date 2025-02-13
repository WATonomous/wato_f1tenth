#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class EmergencyBrake(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('emergency_brake')
        
        # Create a subscriber to /cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/teleop',
            self.cmd_vel_callback,
            1000
        )
        
        # Create a publisher to /drive topic
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            1000
        )
        
        # Store current speed and steering angle
        self.current_speed = 0.0
        self.current_steering_angle = 0.0

        # Timer to continuously publish the latest command every 100ms
        self.timer = self.create_timer(0.01, self.publish_drive_command)

        self.get_logger().info("TeleopToAckermann node initialized")
    
    def cmd_vel_callback(self, twist_msg):
        """
        Callback function to process Twist messages and update stored speed/steering.
        """
        if twist_msg.linear.x == 1:  # Forward
            self.current_speed = 1.0  
        elif twist_msg.linear.x == -1:  # Stop
            self.current_speed = 0.0
            self.current_steering_angle = 0.0  

        if twist_msg.angular.z == 1:  # Left turn
            self.current_steering_angle = 0.1  
        elif twist_msg.angular.z == -1:  # Right turn
            self.current_steering_angle = -0.1  

    def publish_drive_command(self):
        """
        Publishes the latest speed and steering values continuously.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.current_speed
        drive_msg.drive.steering_angle = self.current_steering_angle
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    emergency_node = EmergencyBrake()
    rclpy.spin(emergency_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emergency_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()