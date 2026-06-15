import rclpy
from rclpy.node import Node
import subprocess
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class RosbagRecorderNode(Node):
    def __init__(self):
        super().__init__('rosbag_recorder_node')

        # Parameters
        self.declare_parameter('topics', '') 
        self.declare_parameter('output_dir', '')

        topics = self.get_parameter('topics').value
        output_dir = self.get_parameter('output_dir').value

        # Use package directory if no output_dir specified
        if not output_dir:
            package_dir = '/home/bolty/ament_ws/src/rosbag_recorder'
            output_dir = os.path.join(package_dir, 'bags')

        # Create output dir if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # Timestamped folder name
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        output_path = os.path.join(output_dir, f'recording_{timestamp}')

        # Build the command
        if not topics or topics.strip() == '' or topics.strip() == '[]':
            cmd = ['ros2', 'bag', 'record', '-a', '-o', output_path]
        else:
            topic_list = [t.strip().strip("'\"") for t in topics.strip('[]').split(',')]
            cmd = ['ros2', 'bag', 'record'] + topic_list + ['-o', output_path]

        self.get_logger().info(f'Starting rosbag recording to: {output_path}')
        self.get_logger().info(f'Command: {" ".join(cmd)}')

        # Start recording
        self.process = subprocess.Popen(cmd)

    def destroy_node(self):
        # Stop recording cleanly on shutdown
        if self.process:
            self.get_logger().info('Stopping rosbag recording...')
            self.process.terminate()
            self.process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()