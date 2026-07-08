#!/usr/bin/env python3
"""Capture and replay named Nav2 waypoints for the Go2 robot.

The tool is intentionally small and file-based:

  waypoint_tool capture vending --file /root/maps/ven_table_waypoints.yaml
  waypoint_tool capture table --file /root/maps/ven_table_waypoints.yaml
  waypoint_tool list --file /root/maps/ven_table_waypoints.yaml
  waypoint_tool run --only vending table --file /root/maps/ven_table_waypoints.yaml

By default capture uses the current TF transform from map -> base_link, which
works in both live SLAM and AMCL localization once the robot is localized.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_FILE = os.environ.get("GO2_WAYPOINTS", "/root/maps/waypoints.yaml")
DEFAULT_FRAME = "map"
DEFAULT_CHILD_FRAME = "base_link"


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_waypoints(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"frame_id": DEFAULT_FRAME, "waypoints": []}

    with path.open() as f:
        data = yaml.safe_load(f) or {}

    data.setdefault("frame_id", DEFAULT_FRAME)
    data.setdefault("waypoints", [])
    return data


def save_waypoints(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def is_identity_pose(x: float, y: float, yaw: float) -> bool:
    return abs(x) < 0.02 and abs(y) < 0.02 and abs(yaw) < 0.02


class TfPoseGrabber(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_pose_grabber")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def lookup_pose(
        self,
        frame_id: str,
        child_frame_id: str,
        timeout_s: float,
        allow_identity: bool,
    ) -> tuple[float, float, float]:
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        last_error: Exception | None = None
        saw_identity = False

        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.buffer.lookup_transform(
                    frame_id,
                    child_frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exc:
                last_error = exc
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = quat_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
            if not allow_identity and is_identity_pose(translation.x, translation.y, yaw):
                saw_identity = True
                continue
            return (translation.x, translation.y, yaw)

        if saw_identity:
            raise RuntimeError(
                "only received an identity transform; wait for SLAM/Nav2 TF "
                "to settle, or pass --allow-identity if the robot is "
                "intentionally at the map origin"
            )
        if last_error is None:
            raise RuntimeError("timed out before TF lookup returned")
        raise RuntimeError(str(last_error))


class WaypointRunner(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_runner")
        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self._last_feedback_index: int | None = None
        self._last_feedback_time = 0.0

    def log_feedback(self, current_waypoint: int, name_by_index: dict[int, str]) -> None:
        now = time.monotonic()
        if (
            current_waypoint == self._last_feedback_index
            and now - self._last_feedback_time < 5.0
        ):
            return

        print(
            "  navigating to index "
            f"{current_waypoint} "
            f"({name_by_index.get(current_waypoint, 'unknown')})"
        )
        self._last_feedback_index = current_waypoint
        self._last_feedback_time = now


class WaypointMarkerPublisher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("waypoint_marker_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, topic, qos)

    def publish_markers(self, data: dict[str, Any]) -> None:
        self.publisher.publish(self._marker_array(data))

    def _marker_array(self, data: dict[str, Any]) -> MarkerArray:
        frame_id = data["frame_id"]
        stamp = self.get_clock().now().to_msg()
        markers: list[Marker] = []

        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.append(clear)

        for idx, waypoint in enumerate(data["waypoints"]):
            x = float(waypoint["x"])
            y = float(waypoint["y"])
            yaw = float(waypoint["yaw"])
            qx, qy, qz, qw = yaw_to_quat(yaw)

            point = Marker()
            point.header.frame_id = frame_id
            point.header.stamp = stamp
            point.ns = "waypoint_points"
            point.id = idx
            point.type = Marker.SPHERE
            point.action = Marker.ADD
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.08
            point.pose.orientation.w = 1.0
            point.scale.x = 0.25
            point.scale.y = 0.25
            point.scale.z = 0.25
            point.color.r = 0.0
            point.color.g = 0.8
            point.color.b = 1.0
            point.color.a = 1.0
            markers.append(point)

            arrow = Marker()
            arrow.header.frame_id = frame_id
            arrow.header.stamp = stamp
            arrow.ns = "waypoint_heading"
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.18
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.6
            arrow.scale.y = 0.06
            arrow.scale.z = 0.12
            arrow.color.r = 1.0
            arrow.color.g = 0.45
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            markers.append(arrow)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = "waypoint_labels"
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.65
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = str(waypoint["name"])
            markers.append(label)

        return MarkerArray(markers=markers)


def cmd_capture(args: argparse.Namespace) -> int:
    path = Path(args.file)
    data = load_waypoints(path)

    if data["frame_id"] != args.frame:
        print(
            f"error: waypoint file frame is {data['frame_id']!r}, "
            f"but requested frame is {args.frame!r}"
        )
        return 1

    if any(w["name"] == args.name for w in data["waypoints"]):
        print(f"error: waypoint {args.name!r} already exists; remove it first")
        return 1

    rclpy.init()
    node = TfPoseGrabber()
    try:
        x, y, yaw = node.lookup_pose(
            args.frame,
            args.child_frame,
            args.timeout,
            args.allow_identity,
        )
    except RuntimeError as exc:
        print(
            "error: could not capture pose from TF "
            f"{args.frame} -> {args.child_frame}: {exc}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 2

    waypoint = {
        "name": args.name,
        "x": round(x, 4),
        "y": round(y, 4),
        "yaw": round(yaw, 4),
    }
    data["waypoints"].append(waypoint)
    save_waypoints(path, data)

    print(
        f"saved {args.name!r}: x={waypoint['x']}, y={waypoint['y']}, "
        f"yaw={waypoint['yaw']} rad ({math.degrees(yaw):.1f} deg) -> {path}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


def cmd_list(args: argparse.Namespace) -> int:
    path = Path(args.file)
    data = load_waypoints(path)
    waypoints = data["waypoints"]

    if not waypoints:
        print(f"no waypoints in {path}")
        return 0

    print(f"file: {path}")
    print(f"frame_id: {data['frame_id']}")
    print(f"{'name':<24} {'x':>9} {'y':>9} {'yaw(rad)':>10} {'yaw(deg)':>10}")
    for waypoint in waypoints:
        yaw = float(waypoint["yaw"])
        print(
            f"{waypoint['name']:<24} "
            f"{float(waypoint['x']):>9.3f} "
            f"{float(waypoint['y']):>9.3f} "
            f"{yaw:>10.3f} "
            f"{math.degrees(yaw):>10.1f}"
        )
    return 0


def cmd_remove(args: argparse.Namespace) -> int:
    path = Path(args.file)
    data = load_waypoints(path)
    before = len(data["waypoints"])
    data["waypoints"] = [
        waypoint
        for waypoint in data["waypoints"]
        if waypoint["name"] != args.name
    ]

    if len(data["waypoints"]) == before:
        print(f"no waypoint named {args.name!r} in {path}")
        return 1

    save_waypoints(path, data)
    print(f"removed {args.name!r} from {path}")
    return 0


def cmd_clear(args: argparse.Namespace) -> int:
    path = Path(args.file)
    if path.exists():
        path.unlink()
        print(f"deleted {path}")
    else:
        print(f"{path} does not exist")
    return 0


def select_waypoints(
    all_waypoints: list[dict[str, Any]],
    selected_names: list[str] | None,
) -> list[dict[str, Any]]:
    if not selected_names:
        return list(all_waypoints)

    by_name = {waypoint["name"]: waypoint for waypoint in all_waypoints}
    missing = [name for name in selected_names if name not in by_name]
    if missing:
        raise KeyError(", ".join(missing))

    return [by_name[name] for name in selected_names]


def cmd_run(args: argparse.Namespace) -> int:
    path = Path(args.file)
    data = load_waypoints(path)

    if not data["waypoints"]:
        print(f"no waypoints in {path}; capture some first")
        return 1

    try:
        selected = select_waypoints(data["waypoints"], args.only)
    except KeyError as exc:
        print(f"error: unknown waypoint(s): {exc}")
        return 1

    rclpy.init()
    node = WaypointRunner()

    print("waiting for /follow_waypoints action server...")
    if not node.client.wait_for_server(timeout_sec=args.timeout):
        print("error: /follow_waypoints is not available. Is Nav2 active?")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    goal = FollowWaypoints.Goal()
    stamp = node.get_clock().now().to_msg()
    for waypoint in selected:
        pose = PoseStamped()
        pose.header.frame_id = data["frame_id"]
        pose.header.stamp = stamp
        pose.pose.position.x = float(waypoint["x"])
        pose.pose.position.y = float(waypoint["y"])
        qx, qy, qz, qw = yaw_to_quat(float(waypoint["yaw"]))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.poses.append(pose)

    names = [waypoint["name"] for waypoint in selected]
    print(f"sending {len(goal.poses)} waypoint(s): {names}")
    for idx, waypoint in enumerate(selected):
        yaw = float(waypoint["yaw"])
        print(
            f"  [{idx}] {waypoint['name']}: "
            f"x={float(waypoint['x']):.3f}, "
            f"y={float(waypoint['y']):.3f}, "
            f"yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)"
        )

    name_by_index = {
        idx: waypoint["name"]
        for idx, waypoint in enumerate(selected)
    }
    send_future = node.client.send_goal_async(
        goal,
        feedback_callback=lambda feedback: node.log_feedback(
            feedback.feedback.current_waypoint,
            name_by_index,
        ),
    )
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()

    if goal_handle is None or not goal_handle.accepted:
        print("error: waypoint goal rejected by Nav2")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    missed = list(result.missed_waypoints) if result else []

    if missed:
        missed_names = [name_by_index.get(idx, "unknown") for idx in missed]
        print(f"finished, but missed waypoint indices: {missed} ({missed_names})")
    else:
        print("finished; all waypoints reached")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if not missed else 4


def cmd_show(args: argparse.Namespace) -> int:
    path = Path(args.file)
    data = load_waypoints(path)
    if not data["waypoints"]:
        print(f"no waypoints in {path}; capture some first")
        return 1

    rclpy.init()
    node = WaypointMarkerPublisher(args.topic)
    period_s = 1.0 / args.rate

    print(
        f"publishing {len(data['waypoints'])} waypoint marker(s) from {path} "
        f"on {args.topic}; Ctrl-C to stop"
    )
    try:
        while rclpy.ok():
            data = load_waypoints(path)
            node.publish_markers(data)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="waypoint_tool")
    parser.add_argument(
        "--file",
        default=DEFAULT_FILE,
        help=f"waypoint YAML file (default: {DEFAULT_FILE})",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    capture = subparsers.add_parser("capture", help="save current robot pose")
    capture.add_argument("name", help="unique waypoint name")
    capture.add_argument("--frame", default=DEFAULT_FRAME)
    capture.add_argument("--child-frame", default=DEFAULT_CHILD_FRAME)
    capture.add_argument("--timeout", type=float, default=5.0)
    capture.add_argument(
        "--allow-identity",
        action="store_true",
        help="allow capturing x=0, y=0, yaw=0 instead of treating it as stale TF",
    )
    capture.set_defaults(func=cmd_capture)

    list_cmd = subparsers.add_parser("list", help="list saved waypoints")
    list_cmd.set_defaults(func=cmd_list)

    remove = subparsers.add_parser("remove", help="remove a waypoint by name")
    remove.add_argument("name")
    remove.set_defaults(func=cmd_remove)

    clear = subparsers.add_parser("clear", help="delete the waypoint file")
    clear.set_defaults(func=cmd_clear)

    run = subparsers.add_parser("run", help="send waypoints to Nav2")
    run.add_argument("--only", nargs="+", help="run named waypoints in order")
    run.add_argument("--timeout", type=float, default=10.0)
    run.set_defaults(func=cmd_run)

    show = subparsers.add_parser("show", help="publish Foxglove waypoint markers")
    show.add_argument("--topic", default="/waypoint_markers")
    show.add_argument("--rate", type=float, default=1.0)
    show.set_defaults(func=cmd_show)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
