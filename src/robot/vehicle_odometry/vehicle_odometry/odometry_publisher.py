# vehicle_odometry/odometry_publisher.py
# Final version with timestamp synchronization and EKF yaw-rate fusion, integrating fused yaw rate for angle

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import math


class EKFBicycleOdometry(Node):
    def __init__(self):
        super().__init__('ekf_bicycle_odometry')
        # --- parameters ---
        self.declare_parameter('left_encoder_topic', '/autodrive/f1tenth_1/left_encoder')
        self.declare_parameter('right_encoder_topic', '/autodrive/f1tenth_1/right_encoder')
        self.declare_parameter('steering_topic', '/autodrive/f1tenth_1/steering')
        self.declare_parameter('imu_topic', '/autodrive/f1tenth_1/imu')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('wheelbase', 0.324)
        self.declare_parameter('wheel_radius', 0.059)
        self.declare_parameter('CPR', 1920)
        self.declare_parameter('Q', '1e-3 0 0 0; 0 1e-3 0 0; 0 0 1e-4 0; 0 0 0 1e-4')
        self.declare_parameter('R_rate', 1e-4)
        self.declare_parameter('max_steering_angle', math.radians(30.0))
        self.declare_parameter('max_time_diff', 0.02)  # 20ms tolerance

        # Retrieve parameters
        self.Rw = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheelbase').value
        self.CPR = self.get_parameter('CPR').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.max_time_diff = self.get_parameter('max_time_diff').value

        # Parse process noise Q matrix as a full 4x4
        Q_str = self.get_parameter('Q').value
        Q_rows = [[float(x) for x in row.strip().split()] for row in Q_str.split(';')]
        self.Q = np.array(Q_rows)  # 4x4 process noise matrix
        # Measurement noise for yaw-rate
        self.R_rate = self.get_parameter('R_rate').value

        # EKF state: [x, y, yaw, yaw_rate]
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 0.1

        # Encoder state
        self.left_ticks = 0.0
        self.right_ticks = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_time = None
        self.latest_encoder_time = None
        self.initialized_encoders = False

        # IMU state
        self.imu_z = None
        self.imu_stamp = None
        self.new_imu_data = False

        # Control input
        self.steering = 0.0
        # Linear velocity
        self.v = 0.0

        # ROS interfaces
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(JointState, self.get_parameter('left_encoder_topic').value, self.left_cb, 10)
        self.create_subscription(JointState, self.get_parameter('right_encoder_topic').value, self.right_cb, 10)
        self.create_subscription(Float32, self.get_parameter('steering_topic').value, self.steering_cb, 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_cb, 10)
        self.timer = self.create_timer(0.02, self.on_timer)

    def left_cb(self, msg):
        if msg.position:
            self.left_ticks = msg.position[0]
            self.latest_encoder_time = msg.header.stamp
            if self.prev_left_ticks is None:
                self.prev_left_ticks = self.left_ticks
            self._initialize_encoders()

    def right_cb(self, msg):
        if msg.position:
            self.right_ticks = msg.position[0]
            self.latest_encoder_time = msg.header.stamp
            if self.prev_right_ticks is None:
                self.prev_right_ticks = self.right_ticks
            self._initialize_encoders()

    def _initialize_encoders(self):
        # Once both prev ticks are set, seed time and mark initialized
        if not self.initialized_encoders and self.prev_left_ticks is not None and self.prev_right_ticks is not None:
            self.initialized_encoders = True
            self.prev_time = self.latest_encoder_time.sec + self.latest_encoder_time.nanosec * 1e-9

    def steering_cb(self, msg):
        # clamp steering to allowable range
        angle = msg.data
        self.steering = max(min(angle, self.max_steering), -self.max_steering)

    def imu_cb(self, msg):
        self.imu_z = msg.angular_velocity.z
        self.imu_stamp = msg.header.stamp
        self.new_imu_data = True

    def on_timer(self):
        # wait until encoders are initialized
        if not self.initialized_encoders or self.latest_encoder_time is None:
            return

        # Use encoder header time for dt
        t = self.latest_encoder_time.sec + self.latest_encoder_time.nanosec * 1e-9
        if self.prev_time is not None and t <= self.prev_time:
            return

        # EKF predict + update
        self._update_state(t)
        # Publish with fused state
        self.publish_odometry()
        # clear IMU flag
        self.new_imu_data = False
        self.imu_z = None

    def _update_state(self, t):
        dt = t - self.prev_time if self.prev_time is not None else 0.0

        # Encoder distances
        delta_left = (self.left_ticks - self.prev_left_ticks) * (2 * math.pi * self.Rw) / self.CPR
        delta_right = (self.right_ticks - self.prev_right_ticks) * (2 * math.pi * self.Rw) / self.CPR
        self.v = (delta_left + delta_right) / (2.0 * dt)

        # Bicycle model yaw-rate prediction
        yaw_dot_model = (self.v / self.L) * math.tan(self.steering)
        x_prior = self.x.flatten()

        # --- EKF Predict ---
        # Predict position using last filtered yaw rate
        x_pred = np.array([
            x_prior[0] + self.v * math.cos(x_prior[2]) * dt,
            x_prior[1] + self.v * math.sin(x_prior[2]) * dt,
            x_prior[2] + x_prior[3] * dt,
            yaw_dot_model
        ]).reshape(-1, 1)

        # State Jacobian
        F = np.eye(4)
        F[0, 2] = -self.v * math.sin(x_prior[2]) * dt
        F[1, 2] = self.v * math.cos(x_prior[2]) * dt
        F[2, 3] = dt

        # Covariance predict
        P_pred = F @ self.P @ F.T + self.Q

        # --- EKF Update: fuse IMU yaw-rate ---
        if self.new_imu_data and self.imu_stamp is not None:
            imu_time = self.imu_stamp.sec + self.imu_stamp.nanosec * 1e-9
            if abs(imu_time - t) < self.max_time_diff:
                # measurement matrix for yaw-rate
                H = np.array([[0, 0, 0, 1]])
                z = np.array([[self.imu_z]])
                R = np.array([[self.R_rate]])
                # residual
                y = z - H @ x_pred
                S = H @ P_pred @ H.T + R
                K = P_pred @ H.T @ np.linalg.inv(S)
                # update state & cov
                x_upd = x_pred + K @ y
                P_upd = (np.eye(4) - K @ H) @ P_pred
                self.x = x_upd
                self.P = P_upd
                # After update, integrate yaw using the fused yaw rate
                self.x[2, 0] = x_prior[2] + self.x[3, 0] * dt
            else:
                self.get_logger().warn(f"IMU time diff: {abs(imu_time - t):.3f}s, skipping IMU update")
                self.x = x_pred
                self.P = P_pred
        else:
            self.x = x_pred
            self.P = P_pred

        self.x[2, 0] = x_prior[2] + self.x[3, 0] * dt

        # update history
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks
        self.prev_time = t

    def publish_odometry(self):
        x, y, yaw, yaw_dot = self.x.flatten()
        odom = Odometry()
        odom.header.stamp = self.latest_encoder_time
        odom.header.frame_id = self.get_parameter('frame_id').value
        odom.child_frame_id = self.get_parameter('child_frame_id').value

        # Pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)

        # Twist
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = yaw_dot

        self.odom_pub.publish(odom)

        # TF
        tfs = TransformStamped()
        tfs.header.stamp = odom.header.stamp
        tfs.header.frame_id = odom.header.frame_id
        tfs.child_frame_id = odom.child_frame_id
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = EKFBicycleOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
