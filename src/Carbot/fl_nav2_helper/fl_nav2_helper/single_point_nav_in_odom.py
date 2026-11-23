#!/usr/bin/env python3

import os
import signal
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion


class RelayTopics(Node):

    def __init__(self):
        super().__init__('relay_topics')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('sensor_offset', 0.0)
        self.declare_parameter('do_offset', True)
        self.declare_parameter('pub_odom_tf', True)

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.sensor_offset = float(self.get_parameter('sensor_offset').value)
        self.do_offset = bool(self.get_parameter('do_offset').value)
        self.pub_odom_tf = bool(self.get_parameter('pub_odom_tf').value)

        self.get_logger().info('Using parameters:')
        self.get_logger().info(f'  sensor_offset: {self.sensor_offset}')
        self.get_logger().info(f'  do_offset: {self.do_offset}')
        self.get_logger().info(f'  pub_odom_tf: {self.pub_odom_tf}')

        # -------------------------
        # State
        # -------------------------
        self.got_imu = False
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        self.since_start_stamp = time.time()

        self.kill_node_list = [
            'ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml',
            'fastlio_mapping --ros-args --params-file',
        ]

        self.br = TransformBroadcaster(self)

        # -------------------------
        # QoS
        # -------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -------------------------
        # Subscribers
        # -------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            odom_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar_scan',
            self.lidar_scan_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            sensor_qos
        )

        # -------------------------
        # Publishers
        # -------------------------
        self.lidar_odom_pub = self.create_publisher(
            Odometry,
            '/lidar_odom',
            odom_qos
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            sensor_qos
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/lidar_imu',
            sensor_qos
        )

        self.get_logger().info('relay_topics node started')

    # -------------------------
    # Parameter callback
    # -------------------------
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'sensor_offset' and param.type_ in (
                Parameter.Type.DOUBLE,
                Parameter.Type.INTEGER
            ):
                self.sensor_offset = float(param.value)

            elif param.name == 'do_offset' and param.type_ == Parameter.Type.BOOL:
                self.do_offset = bool(param.value)

            elif param.name == 'pub_odom_tf' and param.type_ == Parameter.Type.BOOL:
                self.pub_odom_tf = bool(param.value)

        self.get_logger().info('Parameters updated')
        self.get_logger().info(f'  sensor_offset: {self.sensor_offset}')
        self.get_logger().info(f'  do_offset: {self.do_offset}')
        self.get_logger().info(f'  pub_odom_tf: {self.pub_odom_tf}')

        return SetParametersResult(successful=True)

    # -------------------------
    # LaserScan relay
    # -------------------------
    def lidar_scan_callback(self, msg: LaserScan):
        scan_msg = LaserScan()
        scan_msg = msg

        # Keep original timestamp for TF consistency
        scan_msg.header.stamp = msg.header.stamp

        # Force output frame for slam_toolbox
        scan_msg.header.frame_id = 'laser_frame'

        self.scan_pub.publish(scan_msg)

    # -------------------------
    # IMU relay
    # -------------------------
    def imu_callback(self, msg: Imu):
        self.got_imu = True

        self.gx = msg.angular_velocity.x
        self.gy = msg.angular_velocity.y
        self.gz = msg.angular_velocity.z

        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z

    # -------------------------
    # Odometry relay + TF publish
    # -------------------------
    def odom_callback(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        x_l = msg.pose.pose.position.x
        y_l = msg.pose.pose.position.y
        z_l = msg.pose.pose.position.z

        x_b = x_l
        y_b = y_l

        if self.do_offset:
            q_list = [qx, qy, qz, qw]
            (_, _, yaw) = euler_from_quaternion(q_list)

            x_b = x_l - self.sensor_offset * math.cos(yaw)
            y_b = y_l - self.sensor_offset * math.sin(yaw)

        # Publish relayed odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist

        odom_msg.pose.pose.position.x = x_b
        odom_msg.pose.pose.position.y = y_b

        self.lidar_odom_pub.publish(odom_msg)

        # Publish odom -> base_link TF
        if self.pub_odom_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x_b
            t.transform.translation.y = y_b
            t.transform.translation.z = z_l
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)

        # Republish IMU with orientation from odometry if available
        if self.got_imu:
            imu_msg = Imu()
            imu_msg.header.stamp = msg.header.stamp
            imu_msg.header.frame_id = 'imu_link'

            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            imu_msg.angular_velocity.x = self.gx
            imu_msg.angular_velocity.y = self.gy
            imu_msg.angular_velocity.z = self.gz

            imu_msg.linear_acceleration.x = self.ax
            imu_msg.linear_acceleration.y = self.ay
            imu_msg.linear_acceleration.z = self.az

            self.imu_pub.publish(imu_msg)

        # Detect abnormal jump during startup
        if (time.time() - self.since_start_stamp) < 300.0:
            if abs(x_l) > 100.0 or abs(y_l) > 100.0 or abs(z_l) > 100.0:
                self.get_logger().warn('Position jump error detected, restarting fast-lio')
                self.kill_fastlio_processes()

    # -------------------------
    # Kill helper
    # -------------------------
    def kill_fastlio_processes(self):
        for node_to_kill in self.kill_node_list:
            for line in os.popen(f'ps ax | grep "{node_to_kill}" | grep -v grep'):
                fields = line.split()
                if not fields:
                    continue
                pid = int(fields[0])
                try:
                    os.kill(pid, signal.SIGKILL)
                    self.get_logger().warn(f'Killed process {pid}')
                except Exception as e:
                    self.get_logger().error(f'Failed to kill process {pid}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RelayTopics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
