#!/usr/bin/env python3
"""Stair + platform stability test.

Spawn at x=0 (in front of stairs at x=2.4), run forward to climb 8 stairs (10cm each),
then turn left and walk to the left platform at y=4.

Records per-phase height/pitch/roll stats and reports stability metrics.
"""

import math, time, statistics, sys
import rclpy
from rclpy.node import Node
from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry


class StairPlatformTest(Node):
    def __init__(self):
        super().__init__("stair_platform_test")
        self.pub = self.create_publisher(Inputs, "control_input", 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.heights, self.pitches, self.rolls, self.xs, self.ys = [], [], [], [], []
        self.phase_heights = {"flat": [], "stair_climb": [], "stair_top": [], "platform": []}
        self.t0 = time.time()
        self.phase = "stand"
        self.phase_start = self.t0
        self.failed = False
        self.fail_reason = ""
        self.checkpoints = {k: False for k in ["stair_base", "stair_mid", "stair_top", "platform"]}

        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info("[StairTest] started")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z

        self.heights.append(z)
        self.pitches.append(pitch)
        self.rolls.append(roll)
        self.xs.append(x)
        self.ys.append(y)

        # Phase height buckets
        if x < 2.2:
            self.phase_heights["flat"].append(z)
        elif 2.2 <= x < 4.0:
            self.phase_heights["stair_climb"].append(z)
        elif x >= 4.0 and y < 2.0:
            self.phase_heights["stair_top"].append(z)
        elif y >= 2.0:
            self.phase_heights["platform"].append(z)

        # Checkpoints
        t = time.time() - self.t0
        if x > 2.2 and not self.checkpoints["stair_base"]:
            self.checkpoints["stair_base"] = True
            self.get_logger().info(f"[{t:.1f}s] stair_base x={x:.2f} z={z:.3f}")
        if x > 3.2 and not self.checkpoints["stair_mid"]:
            self.checkpoints["stair_mid"] = True
            self.get_logger().info(f"[{t:.1f}s] stair_mid x={x:.2f} z={z:.3f}")
        if x > 4.2 and not self.checkpoints["stair_top"]:
            self.checkpoints["stair_top"] = True
            self.get_logger().info(f"[{t:.1f}s] stair_top x={x:.2f} z={z:.3f}")
        if y > 3.0 and not self.checkpoints["platform"]:
            self.checkpoints["platform"] = True
            self.get_logger().info(f"[{t:.1f}s] platform x={x:.2f} y={y:.2f} z={z:.3f}")

        # Failure (only check after 5s startup grace period)
        if t > 5.0:
            if abs(roll) > 0.78 or abs(pitch) > 0.78:
                self.failed = True
                self.fail_reason = f"fell: roll={roll:.2f} pitch={pitch:.2f}"
            if z < 0.10:
                self.failed = True
                self.fail_reason = f"base low: z={z:.3f}"

    def send(self, cmd, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        m = Inputs(); m.command = int(cmd)
        m.lx, m.ly, m.rx, m.ry = float(lx), float(ly), float(rx), float(ry)
        self.pub.publish(m)

    def timer_cb(self):
        t = time.time() - self.t0
        pt = time.time() - self.phase_start

        if self.failed:
            self.get_logger().error(f"[{t:.1f}s] FAILED: {self.fail_reason}")
            self.dump(); rclpy.shutdown(); return

        if self.phase == "stand":
            self.send(2)
            if pt > 2.5:
                self.phase = "trot"; self.phase_start = time.time(); self.send(3)
                self.get_logger().info(f"[{t:.1f}s] -> trot")
        elif self.phase == "trot":
            self.send(3)
            if pt > 1.5:
                self.phase = "to_stair"; self.phase_start = time.time()
                self.get_logger().info(f"[{t:.1f}s] -> forward to stair")
        elif self.phase == "to_stair":
            # Medium speed with high swing to climb stairs
            self.send(3, ly=0.35)
            if self.checkpoints["stair_top"]:
                self.phase = "turn_left"; self.phase_start = time.time()
                self.get_logger().info(f"[{t:.1f}s] -> turn left to platform")
            elif pt > 30:
                self.get_logger().warn("to_stair TIMEOUT"); self.phase = "turn_left"; self.phase_start = time.time()
        elif self.phase == "turn_left":
            self.send(3, ry=0.7)
            if pt > 4.0:
                self.phase = "to_platform"; self.phase_start = time.time()
                self.get_logger().info(f"[{t:.1f}s] -> forward to platform")
        elif self.phase == "to_platform":
            self.send(3, ly=0.35)
            if self.checkpoints["platform"] or pt > 15:
                self.phase = "done"; self.phase_start = time.time()
        elif self.phase == "done":
            self.send(3, ly=0.0)
            if pt > 2.0:
                self.dump(); rclpy.shutdown(); return

    def _stats(self, vals, name):
        if not vals:
            return f"{name}: no data"
        return (f"{name}: n={len(vals)} mean={statistics.mean(vals):.3f} "
                f"std={statistics.pstdev(vals):.3f} min={min(vals):.3f} max={max(vals):.3f}")

    def dump(self):
        if not self.heights:
            self.get_logger().info("[StairTest] no data"); return
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"[StairTest] SUMMARY checkpoints={self.checkpoints} failed={self.failed}")
        self.get_logger().info(f"  overall: {self._stats(self.heights, 'height')}")
        self.get_logger().info(f"  overall: max|pitch|={max(abs(p) for p in self.pitches):.3f} "
                               f"max|roll|={max(abs(r) for r in self.rolls):.3f}")
        for k, v in self.phase_heights.items():
            self.get_logger().info(f"  phase: {self._stats(v, k)}")
        self.get_logger().info(f"  final x={self.xs[-1]:.2f} y={self.ys[-1]:.2f} z={self.heights[-1]:.3f}")
        self.get_logger().info("=" * 70)


def main():
    rclpy.init()
    node = StairPlatformTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass


if __name__ == "__main__":
    main()
