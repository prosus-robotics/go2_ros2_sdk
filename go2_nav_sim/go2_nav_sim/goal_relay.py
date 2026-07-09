# Copyright (c) 2026, Prosus Robotics
# SPDX-License-Identifier: Apache-2.0
"""Bridge Foxglove's "Publish" (click-drag pose) tool into Nav2 actions.

Foxglove can only *publish topics* - it has no Nav2 action plugin like RViz's
"Nav2 Goal" / waypoint tools. This node turns those published poses into action
goals so you can drive the sim entirely from the browser:

* ``goal_pose``      (PoseStamped) -> NavigateToPose      (go there now)
* ``waypoint_add``   (PoseStamped) -> append to a route   (build a path)
* ``waypoints_run``  (Empty)       -> FollowWaypoints      (run the route)
* ``waypoints_clear``(Empty)       -> clear the route

Accumulated waypoints are echoed as ``nav_sim/waypoint_markers`` (numbered
spheres) so you can see the route you are drawing in the 3D panel.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose, FollowWaypoints


class GoalRelay(Node):
    def __init__(self) -> None:
        super().__init__('goal_relay')

        self.global_frame = self.declare_parameter('global_frame', 'map').value

        self._nav_to_pose = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_wp = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self._waypoints: list[PoseStamped] = []

        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal, 10)
        self.create_subscription(PoseStamped, 'waypoint_add', self._on_add, 10)
        self.create_subscription(Empty, 'waypoints_run', self._on_run, 10)
        self.create_subscription(Empty, 'waypoints_clear', self._on_clear, 10)

        self._marker_pub = self.create_publisher(MarkerArray, 'nav_sim/waypoint_markers', 1)

        self.get_logger().info(
            "goal_relay up. Foxglove Publish tool -> 'goal_pose' (go now) or "
            "'waypoint_add' (build route) + 'waypoints_run'/'waypoints_clear'."
        )

    # --- single immediate goal ---
    def _on_goal(self, msg: PoseStamped) -> None:
        pose = self._normalize(msg)
        if not self._nav_to_pose.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose action server not available.')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_to_pose.send_goal_async(goal)
        self.get_logger().info(
            f"NavigateToPose -> ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )

    # --- route building ---
    def _on_add(self, msg: PoseStamped) -> None:
        self._waypoints.append(self._normalize(msg))
        self.get_logger().info(f"waypoint #{len(self._waypoints)} added.")
        self._publish_markers()

    def _on_run(self, _msg: Empty) -> None:
        if not self._waypoints:
            self.get_logger().warn('waypoints_run with no waypoints - add some first.')
            return
        if not self._follow_wp.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('follow_waypoints action server not available.')
            return
        goal = FollowWaypoints.Goal()
        goal.poses = list(self._waypoints)
        self._follow_wp.send_goal_async(goal)
        self.get_logger().info(f"FollowWaypoints -> {len(self._waypoints)} waypoints.")

    def _on_clear(self, _msg: Empty) -> None:
        self._waypoints.clear()
        self.get_logger().info('waypoints cleared.')
        self._publish_markers()

    # --- helpers ---
    def _normalize(self, msg: PoseStamped) -> PoseStamped:
        """Foxglove may leave the frame blank / stamp at 0; make it map-framed."""
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id or self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose
        # Guarantee a valid (normalized) quaternion.
        q = pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            pose.pose.orientation.w = 1.0
        return pose

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = 'waypoints'
        markers.markers.append(clear)

        for i, wp in enumerate(self._waypoints):
            dot = Marker()
            dot.header.frame_id = self.global_frame
            dot.header.stamp = self.get_clock().now().to_msg()
            dot.ns = 'waypoints'
            dot.id = i
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose = wp.pose
            dot.scale.x = dot.scale.y = dot.scale.z = 0.3
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.1, 0.6, 1.0, 0.9
            markers.markers.append(dot)

            label = Marker()
            label.header.frame_id = self.global_frame
            label.header.stamp = dot.header.stamp
            label.ns = 'waypoints'
            label.id = 1000 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = wp.pose
            label.pose.position.z += 0.4
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(i + 1)
            markers.markers.append(label)

        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalRelay()
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
