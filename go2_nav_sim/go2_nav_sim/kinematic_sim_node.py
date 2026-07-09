# Copyright (c) 2026, Prosus Robotics
# SPDX-License-Identifier: Apache-2.0
"""Tier-0 kinematic simulator for exercising the Go2 Nav2 stack headlessly.

The real driver abstracts the dog as ``cmd_vel_out -> motion`` and returns
``odom`` + ``/scan``. This node stands in for the whole robot + LiDAR pipeline:

* subscribes ``cmd_vel_out`` (the twist_mux output the real driver consumes),
* integrates a planar holonomic model into a **true** world pose,
* publishes ``odom`` + the ``odom->base_link`` TF (optionally with injected
  drift, so localization/EKF behaviour can be stressed),
* raycasts the ``/map`` occupancy grid from the true pose to synthesise a
  ``sensor_msgs/LaserScan`` matching the real scan geometry,
* seeds AMCL once with the known start pose.

Everything runs in wall-clock time, so the real ``nav2_params.yaml``
(``use_sim_time: False``) is used unchanged - no ``/clock`` needed.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import (
    Twist,
    TransformStamped,
    Quaternion,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KinematicSimNode(Node):
    """Fake base + map-raycast laser for Nav2 testing without Gazebo."""

    def __init__(self) -> None:
        super().__init__('nav_sim')

        # --- Parameters ---
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel_out').value
        self.odom_topic = self.declare_parameter('odom_topic', 'odom').value
        self.scan_topic = self.declare_parameter('scan_topic', 'scan').value
        self.map_topic = self.declare_parameter('map_topic', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.scan_frame = self.declare_parameter('scan_frame', 'base_link').value

        self.start_x = float(self.declare_parameter('start_x', 0.0).value)
        self.start_y = float(self.declare_parameter('start_y', 0.0).value)
        self.start_yaw = float(self.declare_parameter('start_yaw', 0.0).value)

        self.update_rate = float(self.declare_parameter('update_rate', 50.0).value)
        self.scan_rate = float(self.declare_parameter('scan_rate', 12.0).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)

        # Acceleration model. A kinematic sim that applies cmd_vel INSTANTLY makes
        # DWB (which predicts trajectories using acc limits) badly overshoot and
        # oscillate. Ramping actual velocity toward the command at these limits
        # (match nav2_params acc_lim_*) makes the sim behave like the real robot.
        self.accel_x = float(self.declare_parameter('accel_x', 1.5).value)
        self.accel_y = float(self.declare_parameter('accel_y', 2.5).value)
        self.accel_theta = float(self.declare_parameter('accel_theta', 2.0).value)
        self.publish_robot_marker = bool(self.declare_parameter('publish_robot_marker', True).value)

        self.num_beams = int(self.declare_parameter('num_beams', 360).value)
        self.angle_min = float(self.declare_parameter('angle_min', -math.pi).value)
        self.angle_max = float(self.declare_parameter('angle_max', math.pi).value)
        self.range_min = float(self.declare_parameter('range_min', 0.1).value)
        self.range_max = float(self.declare_parameter('range_max', 20.0).value)
        self.use_inf = bool(self.declare_parameter('use_inf', True).value)
        self.occupied_threshold = int(self.declare_parameter('occupied_threshold', 65).value)

        # Injected odom drift (per-sqrt-second random walk). 0.0 => perfect odom.
        self.odom_xy_noise_std = float(self.declare_parameter('odom_xy_noise_std', 0.0).value)
        self.odom_yaw_noise_std = float(self.declare_parameter('odom_yaw_noise_std', 0.0).value)
        self.random_seed = int(self.declare_parameter('random_seed', 0).value)

        self.publish_initialpose = bool(self.declare_parameter('publish_initialpose', True).value)
        self.initialpose_delay = float(self.declare_parameter('initialpose_delay', 2.0).value)

        self.angle_increment = (self.angle_max - self.angle_min) / float(self.num_beams)
        self._rng = np.random.default_rng(self.random_seed if self.random_seed >= 0 else None)

        # --- State: separate "true" (world) and "odom" (drifting) poses ---
        self.true_x, self.true_y, self.true_yaw = self.start_x, self.start_y, self.start_yaw
        self.odom_x, self.odom_y, self.odom_yaw = self.start_x, self.start_y, self.start_yaw
        self.cmd = (0.0, 0.0, 0.0)
        self.cur_vx, self.cur_vy, self.cur_wz = 0.0, 0.0, 0.0  # actual (rate-limited) velocity
        self.last_cmd_time = self.get_clock().now()
        self.last_tick = self.get_clock().now()

        # --- Map ---
        self.map_grid = None
        self.map_res = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_w = 0
        self.map_h = 0

        # --- QoS ---
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # --- I/O ---
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 5)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.marker_pub = self.create_publisher(MarkerArray, 'nav_sim/robot_model', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(PoseStamped, 'sim/reset_pose', self._on_reset, 1)

        self.create_timer(1.0 / self.update_rate, self._on_update)
        self.create_timer(1.0 / self.scan_rate, self._on_scan)
        if self.publish_robot_marker:
            self.create_timer(0.2, self._publish_robot_marker)
        if self.publish_initialpose:
            self._initialpose_timer = self.create_timer(self.initialpose_delay, self._seed_initialpose)

        self.get_logger().info(
            f"nav_sim up: cmd='{self.cmd_vel_topic}' start=({self.start_x:.2f},"
            f"{self.start_y:.2f},{self.start_yaw:.2f}) beams={self.num_beams} "
            f"drift(xy={self.odom_xy_noise_std}, yaw={self.odom_yaw_noise_std})"
        )

    # --- Callbacks ---
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_res = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_w = msg.info.width
        self.map_h = msg.info.height
        self.map_grid = np.array(msg.data, dtype=np.int16).reshape(self.map_h, self.map_w)
        self.get_logger().info(
            f"map: {self.map_w}x{self.map_h} @ {self.map_res:.3f} m/px, "
            f"origin=({self.map_origin_x:.2f},{self.map_origin_y:.2f})"
        )

    def _on_reset(self, msg: PoseStamped) -> None:
        """Teleport the true (and odom) pose - used to isolate scenario runs."""
        self.true_x = msg.pose.position.x
        self.true_y = msg.pose.position.y
        self.true_yaw = yaw_from_quaternion(msg.pose.orientation)
        self.odom_x, self.odom_y, self.odom_yaw = self.true_x, self.true_y, self.true_yaw
        self.cmd = (0.0, 0.0, 0.0)
        self.cur_vx, self.cur_vy, self.cur_wz = 0.0, 0.0, 0.0
        self.get_logger().info(
            f"reset pose -> ({self.true_x:.2f}, {self.true_y:.2f}, {self.true_yaw:.2f})"
        )

    def _on_update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0.0 or dt > 1.0:
            return

        tvx, tvy, twz = self.cmd
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            tvx, tvy, twz = 0.0, 0.0, 0.0

        # Rate-limit actual velocity toward the command (models robot inertia so
        # DWB's acceleration-aware trajectory prediction matches what happens).
        self.cur_vx = self._ramp(self.cur_vx, tvx, self.accel_x * dt)
        self.cur_vy = self._ramp(self.cur_vy, tvy, self.accel_y * dt)
        self.cur_wz = self._ramp(self.cur_wz, twz, self.accel_theta * dt)
        vx, vy, wz = self.cur_vx, self.cur_vy, self.cur_wz

        # True (world) motion.
        self.true_yaw = self._wrap(self.true_yaw + wz * dt)
        self.true_x += (vx * math.cos(self.true_yaw) - vy * math.sin(self.true_yaw)) * dt
        self.true_y += (vx * math.sin(self.true_yaw) + vy * math.cos(self.true_yaw)) * dt

        # Odom motion = same command with optional injected drift on the deltas.
        d_yaw = wz * dt
        d_lin_x = vx * dt
        d_lin_y = vy * dt
        if self.odom_yaw_noise_std > 0.0:
            d_yaw += self._rng.normal(0.0, self.odom_yaw_noise_std * math.sqrt(dt))
        if self.odom_xy_noise_std > 0.0:
            s = self.odom_xy_noise_std * math.sqrt(dt)
            d_lin_x += self._rng.normal(0.0, s)
            d_lin_y += self._rng.normal(0.0, s)
        self.odom_yaw = self._wrap(self.odom_yaw + d_yaw)
        self.odom_x += d_lin_x * math.cos(self.odom_yaw) - d_lin_y * math.sin(self.odom_yaw)
        self.odom_y += d_lin_x * math.sin(self.odom_yaw) + d_lin_y * math.cos(self.odom_yaw)

        self._publish_odom(now, vx, vy, wz)

    def _on_scan(self) -> None:
        if self.map_grid is None:
            return
        ranges = self._raycast(self.true_x, self.true_y, self.true_yaw)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + self.angle_increment * (self.num_beams - 1)
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)

    # --- Helpers ---
    def _publish_odom(self, now, vx: float, vy: float, wz: float) -> None:
        stamp = now.to_msg()
        quat = yaw_to_quaternion(self.odom_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation = quat
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.03
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.03
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf)

    def _raycast(self, x: float, y: float, yaw: float) -> np.ndarray:
        world_ang = yaw + self.angle_min + self.angle_increment * np.arange(self.num_beams)
        step = max(self.map_res * 0.5, 1e-3)
        rs = np.arange(self.range_min, self.range_max, step)

        xs = x + np.cos(world_ang)[:, None] * rs[None, :]
        ys = y + np.sin(world_ang)[:, None] * rs[None, :]
        gx = np.floor((xs - self.map_origin_x) / self.map_res).astype(np.int32)
        gy = np.floor((ys - self.map_origin_y) / self.map_res).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < self.map_w) & (gy >= 0) & (gy < self.map_h)
        gx_c = np.clip(gx, 0, self.map_w - 1)
        gy_c = np.clip(gy, 0, self.map_h - 1)
        occ = in_bounds & (self.map_grid[gy_c, gx_c] >= self.occupied_threshold)

        hit_any = occ.any(axis=1)
        first_idx = occ.argmax(axis=1)
        ranges = np.where(hit_any, rs[first_idx], self.range_max)
        if self.use_inf:
            ranges = np.where(hit_any, ranges, np.inf)
        return ranges.astype(np.float32)

    def _seed_initialpose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.start_x
        msg.pose.pose.position.y = self.start_y
        msg.pose.pose.orientation = yaw_to_quaternion(self.start_yaw)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initialpose_pub.publish(msg)
        self.get_logger().info('Seeded AMCL initial pose from sim start.')
        self._initialpose_timer.cancel()

    def _publish_robot_marker(self) -> None:
        """A simple Go2-shaped body + heading arrow, locked to base_link so it
        rides the TF into map (visible in Foxglove/RViz to show robot pose)."""
        markers = MarkerArray()

        body = Marker()
        body.header.frame_id = self.base_frame
        body.header.stamp = self.get_clock().now().to_msg()
        body.ns = 'go2'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.frame_locked = True
        body.pose.position.z = 0.16
        body.pose.orientation.w = 1.0
        body.scale.x, body.scale.y, body.scale.z = 0.70, 0.31, 0.28
        body.color.r, body.color.g, body.color.b, body.color.a = 0.12, 0.12, 0.13, 0.95
        markers.markers.append(body)

        head = Marker()
        head.header.frame_id = self.base_frame
        head.header.stamp = body.header.stamp
        head.ns = 'go2'
        head.id = 1
        head.type = Marker.ARROW
        head.action = Marker.ADD
        head.frame_locked = True
        head.pose.position.z = 0.16
        head.pose.orientation.w = 1.0
        head.scale.x, head.scale.y, head.scale.z = 0.55, 0.09, 0.09
        head.color.r, head.color.g, head.color.b, head.color.a = 0.95, 0.77, 0.06, 1.0
        markers.markers.append(head)

        self.marker_pub.publish(markers)

    @staticmethod
    def _ramp(current: float, target: float, max_step: float) -> float:
        delta = target - current
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        return current + delta

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
