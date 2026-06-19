#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32       

class ImuPrinter(Node):
    def __init__(self):
        super().__init__('imu_printer')

        self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        self.throttle_pub = self.create_publisher(Float32,'/autodrive/f1tenth_1/throttle_command', 10)
        self.timer = self.create_timer(0.1, self.publish_throttle)

    # ————————————————————————————
    def imu_cb(self, msg: Imu):
        la = msg.linear_acceleration
        self.get_logger().debug(
            f'IMU lin_acc = ({la.x:.2f}, {la.y:.2f}, {la.z:.2f}) [m/s²]'
        )

    # ————————————————————————————
    def publish_throttle(self):
        throttle = Float32()
        throttle.data = 0.5             
        self.throttle_pub.publish(throttle)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
