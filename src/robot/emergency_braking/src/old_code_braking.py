```
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class EmergencyBraking(Node):

    # SETTING UP THE NODE AND SUBSCRIBERS

    def __init__(self):

        # Access the methods, properties of the parent/sibling class and initialize node
        super().__init__('safety_node')

        # Subscriber for laser scan
        self.scan_sub = self.create_subscription(
            # message, topic, method to call when msg received, queue size
            LaserScan, '/scan', self.scan_callback, 10 
            # queue size: will discard the oldest messages when this number is reached
        )

        # Subscriber for odometry (get position and velocity data)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10
        ) 

        # Publisher for braking (control the car's velocity)
        self.brake_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )

        # set current_speed to 0
        self.current_speed = 0.0


    # CALLBACK FUNCTIONS (called when message is received)

    def scan_callback(self, msg):

        # show properties of the message in the terminal
        # self.get_logger().info(f'ranges[0]: {msg.ranges[0]}')
        # self.get_logger().info(f'ranges[5]: {msg.ranges[5]}')
        # self.get_logger().info(f'angle_min: {msg.angle_min}')
        # self.get_logger().info(f'angle_max: {msg.angle_max}')
        # self.get_logger().info(f'range_min: {msg.range_min}')
        # self.get_logger().info(f'range_max: {msg.range_max}')
        # self.get_logger().info(f'angle_increment: {msg.angle_increment}')

        # apply the brake if iTTC is below a specified threshold
        collision_threshold = 0.3
        ittc_values = self.ittc(msg)
        if any(ittc < collision_threshold for ittc in ittc_values):
            self.apply_brake()


    def odom_callback(self, msg):
        # Update the current_speed member variable using velo data from /ego_racecar/odom message
        self.current_speed = msg.twist.twist.linear.x
        self.get_logger().info(f'car\'s velocity: {msg.twist.twist.linear.x}')

    # FUNCTIONS
    
    def ittc(self, msg):
        # list for iTTC values for each distance reading
        ittc_values = []
        # iterate through each beam
        for i, range_val in enumerate(msg.ranges):
            if(range_val > 0): # ensure readings are valid
                # calculate the iTTC for each distance reading
                angle = msg.angle_min + i * msg.angle_increment
                range_rate = self.current_speed * math.cos(angle)
                ittc = range_val / max(range_rate, 0.1)
                # add the iTTC to the list
                ittc_values.append(ittc)
        return ittc_values
    
                
        
    def apply_brake(self):
        self.get_logger().info(f'brake applied')
        # Create an instance of the AckermannDriveStamped message
        brake_msg = AckermannDriveStamped()
        # drive (instance of AckermannDrive that has values such as speed, steering angle)
        # drive.speed (you can set the speed of the car)
        brake_msg.drive.speed = 0.0
        # Publish message
        self.brake_pub.publish(brake_msg)
        

def main(args=None):
    rclpy.init() # initalize ROS2 python client library
    node = SafetyNode() # instance of the node
    rclpy.spin(node) # run the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```