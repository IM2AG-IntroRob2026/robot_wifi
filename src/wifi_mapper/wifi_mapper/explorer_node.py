#!/usr/bin/env python3
"""
explorer_node — drives the Create 3 in a square and triggers WiFi scans.

Square pattern (counter-clockwise):
  start → [N steps +X] → turn 90° left
        → [N steps +Y] → turn 90° left
        → [N steps -X] → turn 90° left
        → [N steps -Y] → done

Uses the Create 3 action servers (drive_distance, rotate_angle) instead of
raw cmd_vel so the robot handles low-level motor control — no PID needed here.
"""

import math
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from rclpy.action import ActionClient
    from irobot_create_msgs.action import DriveDistance, RotateAngle
    ACTIONS_OK = True
except ImportError:
    ACTIONS_OK = False


class ExplorerNode(Node):

    def __init__(self):
        super().__init__('explorer_node')

        self.declare_parameter('steps_per_side', 5)
        self.declare_parameter('step_size',      0.30)   # m between points
        self.declare_parameter('linear_speed',   0.15)   # m/s
        self.declare_parameter('angular_speed',  0.5)    # rad/s
        self.declare_parameter('scan_duration',  3.0)    # s to wait at each point

        p = self.get_parameter
        self.steps   = int(p('steps_per_side').value)
        self.step_sz = float(p('step_size').value)
        self.v_lin   = float(p('linear_speed').value)
        self.v_ang   = float(p('angular_speed').value)
        self.scan_dt = float(p('scan_duration').value)

        self.trigger_pub = self.create_publisher(Bool,   'trigger_scan',     10)
        self.done_pub    = self.create_publisher(Bool,   'exploration_done', 10)
        self.pose_pub    = self.create_publisher(String, 'robot_pose_json',  10)

        if ACTIONS_OK:
            self.drive_client  = ActionClient(self, DriveDistance, 'drive_distance')
            self.rotate_client = ActionClient(self, RotateAngle,   'rotate_angle')
        else:
            self.get_logger().error(
                'irobot_create_msgs not found — '
                'install with: sudo apt install ros-iron-irobot-create-msgs')

        # Integer direction vectors per side — avoids cos/sin floating-point drift.
        # After 4 × 90° turns these stay exact: no accumulated rounding error.
        self._DIRS    = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        self._x       = 0.0
        self._y       = 0.0
        self._wp_done = 0

    # ── Movement ──────────────────────────────────────────────────────

    def _drive(self, distance: float) -> bool:
        """Send a DriveDistance goal and block until the robot finishes."""
        if not ACTIONS_OK:
            return False

        if not self.drive_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('drive_distance server not available')
            return False

        goal = DriveDistance.Goal()
        goal.distance              = float(distance)
        goal.max_translation_speed = float(self.v_lin)

        future = self.drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        if not future.done() or future.result() is None:
            self.get_logger().error('drive: goal send timeout')
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('drive: goal rejected')
            return False

        result_future = handle.get_result_async()
        # 60 s timeout — enough for the slowest expected drive (< 2 m at 0.15 m/s)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            self.get_logger().error('drive: result timeout')
            return False
        return True

    def _rotate(self, angle_rad: float) -> bool:
        """Send a RotateAngle goal and block until done. Positive = CCW (left)."""
        if not ACTIONS_OK:
            return False

        if not self.rotate_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('rotate_angle server not available')
            return False

        goal = RotateAngle.Goal()
        goal.angle              = float(angle_rad)
        goal.max_rotation_speed = float(self.v_ang)

        future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        if not future.done() or future.result() is None:
            self.get_logger().error('rotate: goal send timeout')
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('rotate: goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done():
            self.get_logger().error('rotate: result timeout')
            return False
        return True

    # ── WiFi scan ─────────────────────────────────────────────────────

    def _scan(self):
        """Publish current pose, fire trigger, sleep for scan_duration."""
        pose = String()
        pose.data = json.dumps({'x': round(self._x, 3), 'y': round(self._y, 3)})
        self.pose_pub.publish(pose)

        trig = Bool()
        trig.data = True
        self.trigger_pub.publish(trig)

        total = 4 * self.steps + 1
        pct   = 100 * self._wp_done // max(1, total)
        bar   = '█' * (pct // 5) + '░' * (20 - pct // 5)
        self.get_logger().info(
            f'  ◉ [{bar}] {pct:3d}%  '
            f'point {self._wp_done + 1}/{total}  '
            f'| ({self._x:.2f}, {self._y:.2f}) m')

        self._wp_done += 1
        time.sleep(self.scan_dt)   # wifi_scanner_node measures during this pause

    # ── Main sequence ─────────────────────────────────────────────────

    def run(self):
        """Sequential square traversal — blocking. No separate rclpy.spin() needed."""
        if not ACTIONS_OK:
            self.get_logger().error('Cannot start: irobot_create_msgs missing')
            return

        side_m = self.steps * self.step_sz
        total  = 4 * self.steps + 1

        sep = '═' * 52
        self.get_logger().info(sep)
        self.get_logger().info('  WIFI MAPPER — square traversal')
        self.get_logger().info(f'  Side   : {side_m:.2f} m  ({self.steps} steps × {self.step_sz} m)')
        self.get_logger().info(f'  Points : {total}  (4 sides × {self.steps} + start)')
        self.get_logger().info(f'  Scan   : {self.scan_dt} s/point')
        self.get_logger().info(sep)

        self.get_logger().info('Waiting for action servers...')
        self.drive_client.wait_for_server()
        self.rotate_client.wait_for_server()
        self.get_logger().info('Ready — starting!')

        # Measure at the starting position before moving
        self._scan()

        for side, (dx, dy) in enumerate(self._DIRS):
            self.get_logger().info(f'  ─── Side {side + 1}/4 ───')

            for step in range(self.steps):
                if not self._drive(self.step_sz):
                    self.get_logger().warn(f'  drive failed (side {side+1}, step {step+1}) — continuing')

                # Dead-reckoning with integer direction vectors: no floating-point error
                self._x += dx * self.step_sz
                self._y += dy * self.step_sz

                self._scan()

            # Turn left 90° — skip after the last side (robot is already done)
            if side < 3:
                if not self._rotate(math.pi / 2):
                    self.get_logger().warn(f'  rotate failed after side {side+1} — continuing')

        self.get_logger().info(sep)
        self.get_logger().info(f'  Done — {total} points measured')
        self.get_logger().info(f'  Heatmap: /root/ros2_ws/wifi_results/wifi_heatmap.png')
        self.get_logger().info(sep)

        done = Bool()
        done.data = True
        self.done_pub.publish(done)

        # Spin briefly so the done message is delivered before shutdown
        end = self.get_clock().now().nanoseconds + int(2e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
