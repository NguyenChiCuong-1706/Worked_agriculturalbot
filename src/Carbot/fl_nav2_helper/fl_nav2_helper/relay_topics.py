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

        # Default covariance parameters for relayed odometry / IMU
        self.declare_parameter('odom_pose_cov_diag', [0.05, 0.05, 0.10, 0.10, 0.10, 0.05])
        self.declare_parameter('odom_twist_cov_diag', [0.05, 0.05, 0.10, 0.10, 0.10, 0.05])

        self.declare_parameter('imu_orientation_cov_diag', [0.02, 0.02, 0.02])
        self.declare_parameter('imu_angular_velocity_cov_diag', [0.01, 0.01, 0.01])
        self.declare_parameter('imu_linear_acceleration_cov_diag', [0.10, 0.10, 0.10])

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.sensor_offset = float(self.get_parameter('sensor_offset').value)
        self.do_offset = bool(self.get_parameter('do_offset').value)
        self.pub_odom_tf = bool(self.get_parameter('pub_odom_tf').value)

        self.odom_pose_cov_diag = self._get_diag6_param('odom_pose_cov_diag', [0.05, 0.05, 0.10, 0.10, 0.10, 0.05])
        self.odom_twist_cov_diag = self._get_diag6_param('odom_twist_cov_diag', [0.05, 0.05, 0.10, 0.10, 0.10, 0.05])

        self.imu_orientation_cov_diag = self._get_diag3_param('imu_orientation_cov_diag', [0.02, 0.02, 0.02])
        self.imu_angular_velocity_cov_diag = self._get_diag3_param('imu_angular_velocity_cov_diag', [0.01, 0.01, 0.01])
        self.imu_linear_acceleration_cov_diag = self._get_diag3_param('imu_linear_acceleration_cov_diag', [0.10, 0.10, 0.10])

        self.get_logger().info('Using parameters:')
        self.get_logger().info(f'  sensor_offset: {self.sensor_offset}')
        self.get_logger().info(f'  do_offset: {self.do_offset}')
        self.get_logger().info(f'  pub_odom_tf: {self.pub_odom_tf}')
        self.get_logger().info(f'  odom_pose_cov_diag: {self.odom_pose_cov_diag}')
        self.get_logger().info(f'  odom_twist_cov_diag: {self.odom_twist_cov_diag}')
        self.get_logger().info(f'  imu_orientation_cov_diag: {self.imu_orientation_cov_diag}')
        self.get_logger().info(f'  imu_angular_velocity_cov_diag: {self.imu_angular_velocity_cov_diag}')
        self.get_logger().info(f'  imu_linear_acceleration_cov_diag: {self.imu_linear_acceleration_cov_diag}')

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

        reliable_scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
            reliable_scan_qos
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/lidar_imu',
            sensor_qos
        )

        self.get_logger().info('relay_topics node started')

    # -------------------------
    # Parameter helpers
    # -------------------------
    def _get_diag6_param(self, name, default):
        value = list(self.get_parameter(name).value)
        if len(value) != 6:
            self.get_logger().warn(f'Parameter {name} must have 6 values, using default {default}')
            return list(default)
        return [float(v) for v in value]

    def _get_diag3_param(self, name, default):
        value = list(self.get_parameter(name).value)
        if len(value) != 3:
            self.get_logger().warn(f'Parameter {name} must have 3 values, using default {default}')
            return list(default)
        return [float(v) for v in value]

    def _diag3_to_cov(self, diag_vals):
        return [
            float(diag_vals[0]), 0.0, 0.0,
            0.0, float(diag_vals[1]), 0.0,
            0.0, 0.0, float(diag_vals[2])
        ]

    def _normalize_quaternion(self, qx, qy, qz, qw):
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-12:
            self.get_logger().warn('Received near-zero quaternion norm, using identity quaternion')
            return 0.0, 0.0, 0.0, 1.0
        return qx / norm, qy / norm, qz / norm, qw / norm

    def _is_bad_number(self, v):
        return (not isinstance(v, (int, float))) or math.isnan(v) or math.isinf(v)

    def sanitize_covariance(self, cov, default_diag):
        """
        Ensure ROS 6x6 covariance is valid enough for downstream optimizers:
        - length 36
        - no NaN / inf
        - positive diagonal
        """
        if cov is None or len(cov) != 36:
            out = [0.0] * 36
            for i, v in zip([0, 7, 14, 21, 28, 35], default_diag):
                out[i] = float(v)
            return out

        out = list(cov)

        # Clean invalid entries
        for i in range(36):
            if self._is_bad_number(out[i]):
                out[i] = 0.0

        # Force positive diagonal
        diag_indices = [0, 7, 14, 21, 28, 35]
        for idx, default_val in zip(diag_indices, default_diag):
            if out[idx] <= 0.0:
                out[idx] = float(default_val)

        # Make symmetric by averaging mirrored off-diagonal terms
        # row-major 6x6
        for r in range(6):
            for c in range(r + 1, 6):
                a = r * 6 + c
                b = c * 6 + r
                avg = 0.5 * (out[a] + out[b])
                out[a] = avg
                out[b] = avg

        return out

    def inflate_pose_cov_for_offset(self, cov, offset):
        """
        Applying a rigid offset from sensor origin to base_link introduces
        additional uncertainty in x/y due to heading uncertainty and mounting mismatch.
        This is a practical inflation, not a full Jacobian propagation.
        """
        out = list(cov)

        extra_xy = max(0.005, abs(offset) * 0.02)
        extra_yaw = max(0.002, abs(offset) * 0.005)

        out[0] += extra_xy     # x
        out[7] += extra_xy     # y
        out[35] += extra_yaw   # yaw

        return out

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

            elif param.name == 'odom_pose_cov_diag':
                try:
                    values = [float(v) for v in param.value]
                    if len(values) == 6:
                        self.odom_pose_cov_diag = values
                except Exception:
                    pass

            elif param.name == 'odom_twist_cov_diag':
                try:
                    values = [float(v) for v in param.value]
                    if len(values) == 6:
                        self.odom_twist_cov_diag = values
                except Exception:
                    pass

            elif param.name == 'imu_orientation_cov_diag':
                try:
                    values = [float(v) for v in param.value]
                    if len(values) == 3:
                        self.imu_orientation_cov_diag = values
                except Exception:
                    pass

            elif param.name == 'imu_angular_velocity_cov_diag':
                try:
                    values = [float(v) for v in param.value]
                    if len(values) == 3:
                        self.imu_angular_velocity_cov_diag = values
                except Exception:
                    pass

            elif param.name == 'imu_linear_acceleration_cov_diag':
                try:
                    values = [float(v) for v in param.value]
                    if len(values) == 3:
                        self.imu_linear_acceleration_cov_diag = values
                except Exception:
                    pass

        self.get_logger().info('Parameters updated')
        self.get_logger().info(f'  sensor_offset: {self.sensor_offset}')
        self.get_logger().info(f'  do_offset: {self.do_offset}')
        self.get_logger().info(f'  pub_odom_tf: {self.pub_odom_tf}')
        self.get_logger().info(f'  odom_pose_cov_diag: {self.odom_pose_cov_diag}')
        self.get_logger().info(f'  odom_twist_cov_diag: {self.odom_twist_cov_diag}')
        self.get_logger().info(f'  imu_orientation_cov_diag: {self.imu_orientation_cov_diag}')
        self.get_logger().info(f'  imu_angular_velocity_cov_diag: {self.imu_angular_velocity_cov_diag}')
        self.get_logger().info(f'  imu_linear_acceleration_cov_diag: {self.imu_linear_acceleration_cov_diag}')

        return SetParametersResult(successful=True)

    # -------------------------
    # LaserScan relay
    # -------------------------
    def lidar_scan_callback(self, msg: LaserScan):
        scan_msg = LaserScan()
        scan_msg.header = msg.header
        scan_msg.angle_min = msg.angle_min
        scan_msg.angle_max = msg.angle_max
        scan_msg.angle_increment = msg.angle_increment
        scan_msg.time_increment = msg.time_increment
        scan_msg.scan_time = msg.scan_time
        scan_msg.range_min = msg.range_min
        scan_msg.range_max = msg.range_max
        scan_msg.ranges = msg.ranges
        scan_msg.intensities = msg.intensities

        # Keep original timestamp for TF consistency
        scan_msg.header.stamp = msg.header.stamp

        # Force output frame for slam_toolbox / downstream consumers
        scan_msg.header.frame_id = 'laser_frame'

        self.scan_pub.publish(scan_msg)

    # -------------------------
    # IMU relay input cache
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

        qx, qy, qz, qw = self._normalize_quaternion(qx, qy, qz, qw)

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

        # Build fresh odom message explicitly
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Pose
        odom_msg.pose.pose.position.x = x_b
        odom_msg.pose.pose.position.y = y_b
        odom_msg.pose.pose.position.z = z_l
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Twist
        odom_msg.twist.twist.linear.x = msg.twist.twist.linear.x
        odom_msg.twist.twist.linear.y = msg.twist.twist.linear.y
        odom_msg.twist.twist.linear.z = msg.twist.twist.linear.z
        odom_msg.twist.twist.angular.x = msg.twist.twist.angular.x
        odom_msg.twist.twist.angular.y = msg.twist.twist.angular.y
        odom_msg.twist.twist.angular.z = msg.twist.twist.angular.z

        # Covariance sanitization
        pose_cov = self.sanitize_covariance(
            msg.pose.covariance,
            self.odom_pose_cov_diag
        )
        twist_cov = self.sanitize_covariance(
            msg.twist.covariance,
            self.odom_twist_cov_diag
        )

        # Inflate pose covariance slightly if offset is applied
        if self.do_offset and abs(self.sensor_offset) > 1e-9:
            pose_cov = self.inflate_pose_cov_for_offset(pose_cov, self.sensor_offset)

        odom_msg.pose.covariance = pose_cov
        odom_msg.twist.covariance = twist_cov

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

        # Republish IMU using odom orientation + raw IMU gyro/accel
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

            imu_msg.orientation_covariance = self._diag3_to_cov(
                self.imu_orientation_cov_diag
            )
            imu_msg.angular_velocity_covariance = self._diag3_to_cov(
                self.imu_angular_velocity_cov_diag
            )
            imu_msg.linear_acceleration_covariance = self._diag3_to_cov(
                self.imu_linear_acceleration_cov_diag
            )

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
