"""Automated obstacle traversal test.

Launch gazebo with stair10cm_obst8 world, then send:
  t=0s    : command=2 (stand)
  t=3s    : command=3 (trot)
  t=5s    : start moving forward (ly=0.6)
  Wait until robot passes the 8cm obstacle at x=-1.2, then evaluate
  base height stability and whether the robot falls over.

Publishes control_input_msgs/Inputs.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class ObstacleTestNode(Node):
    def __init__(self):
        super().__init__("obstacle_test_node")

        self.pub = self.create_publisher(Inputs, "control_input", 10)

        self.base_heights = []
        self.base_pitches = []
        self.base_rolls = []
        self.x_positions = []
        self.phase = "stand"  # stand -> trot -> forward -> obstacle -> stairs -> done
        self.phase_start = time.time()
        self.t0 = time.time()
        self.last_log_time = 0.0
        self.obstacle_passed = False
        self.stairs_reached = False
        self.failed = False
        self.fail_reason = ""

        # Subscribe to odom for base state monitoring
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # Publish at 50Hz
        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info("[ObstacleTest] Node started. Waiting for odom...")

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw/pitch/roll from quaternion
        # using tf transformations equivalent
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.base_heights.append(msg.pose.pose.position.z)
        self.base_pitches.append(pitch)
        self.base_rolls.append(roll)
        self.x_positions.append(msg.pose.pose.position.x)

        # Failure detection: fell over (roll or pitch > 45 deg) or height too low
        if abs(roll) > 0.78 or abs(pitch) > 0.78:
            self.failed = True
            self.fail_reason = f"fell over: roll={roll:.2f} pitch={pitch:.2f}"
        if msg.pose.pose.position.z < 0.12:
            self.failed = True
            self.fail_reason = f"base too low: z={msg.pose.pose.position.z:.3f}"

    def send_cmd(self, command, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        msg = Inputs()
        msg.command = int(command)
        msg.lx = float(lx)
        msg.ly = float(ly)
        msg.rx = float(rx)
        msg.ry = float(ry)
        self.pub.publish(msg)

    def timer_cb(self):
        now = time.time()
        t = now - self.t0

        if self.failed:
            self.get_logger().error(f"[ObstacleTest] FAILED at t={t:.1f}s: {self.fail_reason}")
            self.dump_stats(t)
            rclpy.shutdown()
            return

        if self.phase == "stand":
            if t < 0.5:
                self.send_cmd(0)
            elif t < 3.0:
                self.send_cmd(2)  # stand
            else:
                self.get_logger().info(f"[ObstacleTest] t={t:.1f}s -> trot")
                self.phase = "trot"
                self.phase_start = now
                self.send_cmd(3)

        elif self.phase == "trot":
            self.send_cmd(3)
            if (now - self.phase_start) > 2.0:
                self.get_logger().info(f"[ObstacleTest] t={t:.1f}s -> forward")
                self.phase = "forward"
                self.phase_start = now

        elif self.phase == "forward":
            # Move forward at moderate speed toward the 8cm obstacle at x=-1.2
            self.send_cmd(3, ly=0.55)
            if len(self.x_positions) > 0:
                latest_x = self.x_positions[-1]
                if latest_x < -1.0 and not self.obstacle_passed:
                    self.obstacle_passed = True
                    self.get_logger().info(
                        f"[ObstacleTest] t={t:.1f}s -> passed 8cm obstacle region (x={latest_x:.2f})"
                    )
                if latest_x > 2.2 and not self.stairs_reached:
                    self.stairs_reached = True
                    self.get_logger().info(
                        f"[ObstacleTest] t={t:.1f}s -> reached stairs (x={latest_x:.2f})"
                    )
                if latest_x > 3.4:
                    self.get_logger().info(f"[ObstacleTest] t={t:.1f}s -> passed stairs, done")
                    self.phase = "done"
                    self.phase_start = now

        elif self.phase == "done":
            # coast for 2 more seconds for data
            self.send_cmd(3, ly=0.0)
            if (now - self.phase_start) > 2.0:
                self.dump_stats(t)
                rclpy.shutdown()
                return

        # Periodic status log
        if now - self.last_log_time > 1.0 and len(self.base_heights) > 0:
            self.last_log_time = now
            z = self.base_heights[-1]
            pitch = self.base_pitches[-1]
            roll = self.base_rolls[-1]
            x = self.x_positions[-1]
            self.get_logger().info(
                f"[ObstacleTest] t={t:.1f}s phase={self.phase} x={x:.2f} z={z:.3f} "
                f"pitch={pitch:.3f} roll={roll:.3f}"
            )

    def dump_stats(self, t):
        if not self.base_heights:
            self.get_logger().info("[ObstacleTest] no odom data collected")
            return
        import statistics
        z_mean = statistics.mean(self.base_heights)
        z_std = statistics.pstdev(self.base_heights)
        z_min = min(self.base_heights)
        z_max = max(self.base_heights)
        pitch_max = max(abs(p) for p in self.base_pitches)
        roll_max = max(abs(r) for r in self.base_rolls)
        pitch_std = statistics.pstdev(self.base_pitches)
        roll_std = statistics.pstdev(self.base_rolls)
        x_max = max(self.x_positions)

        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"[ObstacleTest] SUMMARY t={t:.1f}s | obstacle_passed={self.obstacle_passed} "
            f"stairs_reached={self.stairs_reached} failed={self.failed}"
        )
        self.get_logger().info(
            f"  Base height : mean={z_mean:.3f} std={z_std:.3f} min={z_min:.3f} max={z_max:.3f}"
        )
        self.get_logger().info(
            f"  Base pitch  : max|pitch|={pitch_max:.3f} std={pitch_std:.3f}"
        )
        self.get_logger().info(
            f"  Base roll   : max|roll|={roll_max:.3f} std={roll_std:.3f}"
        )
        self.get_logger().info(f"  Max forward x: {x_max:.2f}")
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = ObstacleTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
