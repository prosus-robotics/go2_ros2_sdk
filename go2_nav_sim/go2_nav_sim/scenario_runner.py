# Copyright (c) 2026, Prosus Robotics
# SPDX-License-Identifier: Apache-2.0
"""Automated stability scorecard for the Go2 Nav2 stack (Tier-0 sim).

Drives a set of navigation scenarios via nav2_simple_commander and scores the
exact failure modes the nav-stability fixes targeted:

* success / recoveries / time,
* min obstacle clearance (wall-scraping),
* command oscillation (cmd_vel.angular.z sign flips = the overshoot/weave bug),
* final position + heading error.

Each scenario teleports the sim robot to its start (``/sim/reset_pose``) so runs
are isolated and repeatable. Coordinates match the generated ``example.yaml``.

    ros2 run go2_nav_sim scenario_runner
    ros2 run go2_nav_sim scenario_runner --ros-args -p scenarios:="doorway,around_box"
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# (name, start[x,y,yaw], goal[x,y,yaw], thresholds)
SCENARIOS = {
    'straight': {
        'start': (2.0, 2.0, 0.0), 'goal': (8.0, 2.0, 0.0),
        'max_clearance_ok': 0.15, 'max_osc': 10, 'max_xy_err': 0.30, 'max_yaw_err': 0.35,
    },
    'doorway': {
        'start': (3.0, 9.0, 0.0), 'goal': (17.0, 9.0, 0.0),
        'max_clearance_ok': 0.10, 'max_osc': 14, 'max_xy_err': 0.30, 'max_yaw_err': 0.35,
    },
    'around_box': {
        'start': (2.0, 15.0, 0.0), 'goal': (8.0, 15.0, 0.0),
        'max_clearance_ok': 0.12, 'max_osc': 14, 'max_xy_err': 0.30, 'max_yaw_err': 0.35,
    },
    'rotate': {
        'start': (15.0, 15.0, 0.0), 'goal': (15.0, 15.0, 3.0),
        'max_clearance_ok': 0.15, 'max_osc': 12, 'max_xy_err': 0.30, 'max_yaw_err': 0.30,
    },
}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def make_pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    qx, qy, qz, qw = yaw_to_quat(yaw)
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


def yaw_of(pose_msg):
    q = pose_msg.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Monitor:
    """Samples cmd_vel + scan on the navigator node during a run."""

    def __init__(self, nav: Node):
        self.nav = nav
        nav.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        nav.create_subscription(LaserScan, 'scan', self._on_scan, 5)
        self.reset()

    def reset(self):
        self.min_clearance = float('inf')
        self.osc = 0
        self._last_sign = 0

    def _on_cmd(self, msg: Twist):
        wz = msg.angular.z
        sign = 1 if wz > 0.05 else (-1 if wz < -0.05 else 0)
        if sign != 0 and self._last_sign != 0 and sign != self._last_sign:
            self.osc += 1
        if sign != 0:
            self._last_sign = sign

    def _on_scan(self, msg: LaserScan):
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < self.min_clearance:
                self.min_clearance = r


def run_scenario(nav: BasicNavigator, monitor: Monitor, reset_pub, name: str, spec: dict) -> dict:
    sx, sy, syaw = spec['start']
    gx, gy, gyaw = spec['goal']

    # Teleport the sim robot and re-seed AMCL to the scenario start.
    start_pose = make_pose(sx, sy, syaw, nav.get_clock().now().to_msg())
    for _ in range(3):
        reset_pub.publish(start_pose)
        time.sleep(0.1)
    nav.setInitialPose(start_pose)
    time.sleep(2.0)
    monitor.reset()

    goal = make_pose(gx, gy, gyaw, nav.get_clock().now().to_msg())
    nav.goToPose(goal)

    last_pose = None
    recoveries = 0
    t0 = time.time()
    timeout_s = 120.0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None:
            last_pose = fb.current_pose.pose
            recoveries = fb.number_of_recoveries
        if time.time() - t0 > timeout_s:
            nav.cancelTask()
            break

    result = nav.getResult()
    succeeded = result == TaskResult.SUCCEEDED

    if last_pose is not None:
        xy_err = math.hypot(last_pose.position.x - gx, last_pose.position.y - gy)
        yaw_err = abs(math.atan2(math.sin(yaw_of(last_pose) - gyaw),
                                 math.cos(yaw_of(last_pose) - gyaw)))
    else:
        xy_err = float('nan')
        yaw_err = float('nan')

    clearance = monitor.min_clearance if math.isfinite(monitor.min_clearance) else float('nan')

    passed = (
        succeeded
        and (math.isnan(clearance) or clearance >= spec['max_clearance_ok'])
        and monitor.osc <= spec['max_osc']
        and (math.isnan(xy_err) or xy_err <= spec['max_xy_err'])
        and (math.isnan(yaw_err) or yaw_err <= spec['max_yaw_err'])
    )

    return {
        'name': name,
        'success': succeeded,
        'time': time.time() - t0,
        'recoveries': recoveries,
        'clearance': clearance,
        'osc': monitor.osc,
        'xy_err': xy_err,
        'yaw_err': yaw_err,
        'passed': passed,
    }


def print_scorecard(rows):
    hdr = f"{'scenario':<12} {'result':<8} {'t(s)':>6} {'recov':>5} " \
          f"{'clear(m)':>8} {'osc':>4} {'xy_err':>7} {'yaw_err':>7}  verdict"
    print('\n' + hdr)
    print('-' * len(hdr))
    for r in rows:
        print(
            f"{r['name']:<12} "
            f"{('OK' if r['success'] else 'FAIL'):<8} "
            f"{r['time']:>6.1f} {r['recoveries']:>5d} "
            f"{r['clearance']:>8.2f} {r['osc']:>4d} "
            f"{r['xy_err']:>7.2f} {r['yaw_err']:>7.2f}  "
            f"{'PASS' if r['passed'] else 'FAIL'}"
        )
    n_pass = sum(1 for r in rows if r['passed'])
    print('-' * len(hdr))
    print(f"TOTAL: {n_pass}/{len(rows)} scenarios passed\n")


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator()

    param = nav.declare_parameter('scenarios', 'all').value
    selected = list(SCENARIOS.keys()) if param == 'all' else [s.strip() for s in param.split(',')]

    reset_pub = nav.create_publisher(PoseStamped, '/sim/reset_pose', 1)
    monitor = Monitor(nav)

    nav.get_logger().info('Waiting for Nav2 to become active...')
    nav.waitUntilNav2Active()

    rows = []
    for name in selected:
        if name not in SCENARIOS:
            nav.get_logger().warn(f"Unknown scenario '{name}', skipping.")
            continue
        nav.get_logger().info(f"=== Scenario: {name} ===")
        rows.append(run_scenario(nav, monitor, reset_pub, name, SCENARIOS[name]))

    if rows:
        print_scorecard(rows)

    nav.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
