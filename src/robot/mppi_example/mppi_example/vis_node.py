import numpy as np
import os, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from ros_np_multiarray import to_multiarray_f32, to_numpy_f32
from QtMatplotlib import QtPlotter


class Visualizer_Node(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.obs_max_num = 108
        self.reference_max_num = 100
        self.opt_traj_max_num = 10
        self.qt_plotter = QtPlotter()
        self.opt_traj_f = None

        # publishers
        self.reference_pub = self.create_publisher(MarkerArray, "/vis/reference", 1)
        self.opt_traj_pub = self.create_publisher(MarkerArray, "/vis/opt_traj", 1)
        self.obstacle_pub = self.create_publisher(MarkerArray, "/vis/obstacle", 1)
        
        self.frenet_pose_sub = self.create_subscription(Float32MultiArray, "/frenet_pose", self.frenet_pose_callback, 1)
        self.reference_sub = self.create_subscription(Float32MultiArray, "/reference_arr", self.reference_callback, 1)
        self.opt_traj_sub = self.create_subscription(Float32MultiArray, "/opt_traj_arr", self.opt_traj_callback, 1)
        self.obstacle_sub = self.create_subscription(Float32MultiArray, "/obstacle_arr_xy", self.obstacle_callback, 1)
        self.reward_sub = self.create_subscription(Float32MultiArray, "/reward_arr", self.reward_callback, 1)

    def reward_callback(self, arr_msg):
        reward_arr = to_numpy_f32(arr_msg)
        min_values = np.min(reward_arr[:, 0])
        self.qt_plotter.scatter(-reward_arr[:, 1], reward_arr[:, 0] - min_values, c=reward_arr[:, 2], s=20, plot_num=0, live=1)
        if self.opt_traj_f is not None:
            self.qt_plotter.scatter(-self.opt_traj_f[:, 1], self.opt_traj_f[:, 0] - min_values, s=20, plot_num=1, live=1)

    def reference_callback(self, arr_msg):
        reference_arr = to_numpy_f32(arr_msg)
        reference_msg = self.waypoints_to_markerArray(reference_arr, self.reference_max_num, 0, 1, r=0.0, g=0.0, b=1.0)
        self.reference_pub.publish(reference_msg)
        
        
    def frenet_pose_callback(self, arr_msg):
        reference_arr = to_numpy_f32(arr_msg)
        
    def obstacle_callback(self, arr_msg):
        obstacle_arr = to_numpy_f32(arr_msg)
        obstacle_msg = self.waypoints_to_markerArray(obstacle_arr, self.obs_max_num, 0, 1, r=1.0, g=0.0, b=0.0)
        self.obstacle_pub.publish(obstacle_msg)

    def opt_traj_callback(self, arr_msg):
        opt_traj_arr = to_numpy_f32(arr_msg)
        # self.qt_plotter.scatter(opt_traj_arr[:, 0], opt_traj_arr[:, 1], s=20, plot_num=0, live=1)
        opt_traj_msg = self.waypoints_to_markerArray(opt_traj_arr[:10], self.opt_traj_max_num, 0, 1, r=0.0, g=1.0, b=0.0)
        self.opt_traj_f = opt_traj_arr[10:]
        self.opt_traj_pub.publish(opt_traj_msg)

    def waypoints_to_markerArray(self, waypoints, max_num, xind, yind, r=0.0, g=1.0, b=0.0):
        # Publish the reference trajectory
        array = MarkerArray()

        for i in range(max_num):
            message = Marker()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.type= Marker.SPHERE
            message.id=i
            message.pose.orientation.x=0.0
            message.pose.orientation.y=0.0
            message.pose.orientation.z=0.0
            message.pose.orientation.w=1.0
            message.scale.x=0.2
            message.scale.y=0.2
            message.scale.z=0.2
            message.color.a=1.0
            message.color.r=r
            message.color.b=g
            message.color.g=b
            if i < waypoints.shape[0]:
                message.pose.position.x=float(waypoints[i,xind])
                message.pose.position.y=float(waypoints[i,yind])
                message.action = Marker.ADD
            else:
                message.pose.position.x=0.
                message.pose.position.y=0.
                message.action = Marker.DELETE
            message.pose.position.z=0.0
            array.markers.append(message)

        return array

def main(args=None):
    rclpy.init(args=args)
    print("Track Visualizer Mode Initialized")
    lmppi_node = Visualizer_Node()
    rclpy.spin(lmppi_node)

    lmppi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()