# change to c++

import rclpy
from rclpy.node import Node 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Obstacles(Node):

    def __init__(self):
        super().__init__('obstacles')
        # marker publisher
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        # timer
        self.timer = self.create_timer(1.0, self.publish_obstacle)

    def publish_obstacle(self):
        marker = Marker()
        # frame id: coordinate frame. 'map' is usually used for global references
        marker.header.frame_id = 'map'
        # timer to continuously publish obstacles
        marker.header.stamp = self.get_clock().now().to_msg()
        # set up marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=5.0, y=0.0, z=0.0)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0  
        marker.color.g = 0.0  
        marker.color.b = 0.0  
        marker.color.a = 1.0
        # publish marker
        self.marker_pub.publish(marker)
        self.get_logger().info("Publishing obstacle marker")

def main(args=None):
    rclpy.init(args=args)
    node = Obstacles()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()