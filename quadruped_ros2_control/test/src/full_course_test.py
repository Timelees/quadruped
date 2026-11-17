"""Automated full-course test: 8cm obstacle -> stairs -> left platform.

Spawn robot at x=-2.5, face +x. Sequence:
  t=0-3s   : stand
  t=3-5s   : trot
  t=5-12s  : forward toward 8cm obstacle at x=-1.2
  t=12-22s : continue forward up stairs (start at x=2.4)
  t=22-28s : turn left (yaw += 90°) then move toward left platform at y=4
  t=28-40s : forward onto platform

Records base height/pitch/roll throughout. Fails if robot falls over.
"""

import math
import time
import statistics

import rclpy
from rclpy.node import Node

from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry


class FullCourseTestNode(Node):
    def __init__(self):
        super().__init__("full_course_test_node")
        self.pub = self.create_publisher(Inputs, "control_input", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        self.heights, self.pitches, self.rolls, self.xs, self.ys = [], [], [], [], []
        self.t0 = time.time()
        self.phase = "stand"
        self.phase_start = self.t0
        self.failed = False
        self.fail_reason = ""
        self.checkpoints = {
            "obstacle_passed": False,
            "stairs_reached": False,
            "stairs_climbed": False,
            "platform_reached": False,
        }
        self.heights_by_phase = {
            "flat_pre_obstacle": [],
            "obstacle_crossing": [],
            "stairs": [],
            "platform_approach": [],
        }

        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info("[FullCourse] Node started, waiting for odom...")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.heights.append(z)
        self.pitches.append(pitch)
        self.rolls.append(roll)
        self.xs.append(x)
        self.ys.append(y)

        # Per-phase height collection (updated each odom sample)
        if x < -1.4:
            self.heights_by_phase["flat_pre_obstacle"].append(z)
        elif -1.4 <= x < -0.8:
            self.heights_by_phase["obstacle_crossing"].append(z)
        elif 2.0 <= x < 3.6:
            self.heights_by_phase["stairs"].append(z)
        elif y > 2.5:
            self.heights_by_phase["platform_approach"].append(z)

        # Checkpoints
        if x > -0.9 and not self.checkpoints["obstacle_passed"]:
            self.checkpoints["obstacle_passed"] = True
            self.get_logger().info(f"[FullCourse] t={time.time()-self.t0:.1f}s CHECKPOINT: passed 8cm obstacle")
        if x > 2.2 and not self.checkpoints["stairs_reached"]:
            self.checkpoints["stairs_reached"] = True
            self.get_logger().info(f"[FullCourse] t={time.time()-self.t0:.1f}s CHECKPOINT: reached stairs")
        if x > 3.5 and not self.checkpoints["stairs_climbed"]:
            self.checkpoints["stairs_climbed"] = True
            self.get_logger().info(f"[FullCourse] t={time.time()-self.t0:.1f}s CHECKPOINT: climbed stairs")
        if y > 3.3 and x > -1.0 and not self.checkpoints["platform_reached"]:
            self.checkpoints["platform_reached"] = True
            self.get_logger().info(f"[FullCourse] t={time.time()-self.t0:.1f}s CHECKPOINT: reached platform")

        # Failure
        if abs(roll) > 0.78 or abs(pitch) > 0.78:
            self.failed = True
            self.fail_reason = f"fell: roll={roll:.2f} pitch={pitch:.2f}"
        if z < 0.10:
            self.failed = True
            self.fail_reason = f"base too low: z={z:.3f}"

    def send(self, cmd, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        m = Inputs()
        m.command, m.lx, m.ly, m.rx, m.ry = int(cmd), float(lx), float(ly), float(rx), float(ry)
        self.pub.publish(m)

    def timer_cb(self):
        now = time.time()
        t = now - self.t0
        pt = now - self.phase_start

        if self.failed:
            self.get_logger().error(f"[FullCourse] FAILED t={t:.1f}s: {self.fail_reason}")
            self.dump(); rclpy.shutdown(); return

        # Phase state machine
        if self.phase == "stand":
            self.send(2)
            if pt > 3.0:
                self.get_logger().info(f"[FullCourse] t={t:.1f}s -> trot")
                self.phase = "trot"; self.phase_start = now; self.send(3)
        elif self.phase == "trot":
            self.send(3)
            if pt > 2.0:
                self.get_logger().info(f"[FullCourse] t={t:.1f}s -> forward (to obstacle)")
                self.phase = "to_obstacle"; self.phase_start = now
        elif self.phase == "to_obstacle":
            # Forward to cross the 8cm obstacle at x=-1.2
            self.send(3, ly=0.55)
            if self.checkpoints["obstacle_passed"]:
                self.get_logger().info(f"[FullCourse] t={t:.1f}s -> forward (to stairs)")
                self.phase = "to_stairs"; self.phase_start = now
            elif pt > 15.0:
                self.get_logger().warn("[FullCourse] to_obstacle TIMEOUT")
                self.phase = "to_stairs"; self.phase_start = now
        elif self.phase == "to_stairs":
            # Continue forward up stairs
            self.send(3, ly=0.55)
            if self.checkpoints["stairs_climbed"]:
                self.get_logger().info(f"[FullCourse] t={t:.1f}s -> turn left to platform")
                self.phase = "turn_to_platform"; self.phase_start = now
            elif pt > 20.0:
                self.get_logger().warn("[FullCourse] to_stairs TIMEOUT")
                self.phase = "turn_to_platform"; self.phase_start = now
        elif self.phase == "turn_to_platform":
            # Turn left (yaw += 90° via ry) for ~3s
            self.send(3, ry=0.8)
            if pt > 3.5:
                self.get_logger().info(f"[FullCourse] t={t:.1f}s -> forward to platform")
                self.phase = "to_platform"; self.phase_start = now
        elif self.phase == "to_platform":
            # Move forward (ly) to reach platform at y~4
            self.send(3, ly=0.55)
            if self.checkpoints["platform_reached"] or pt > 15.0:
                self.phase = "done"; self.phase_start = now
                self.get_logger().info(f"[FullCourse] t={t:.1f}s -> done")
        elif self.phase == "done":
            self.send(3, ly=0.0)
            if pt > 2.0:
                self.dump(); rclpy.shutdown(); return

    def _phase_stats(self, vals, name):
        if not vals:
            return f"{name}: no data"
        return (f"{name}: n={len(vals)} mean={statistics.mean(vals):.3f} "
                f"std={statistics.pstdev(vals):.3f} "
                f"min={min(vals):.3f} max={max(vals):.3f}")

    def dump(self):
        if not self.heights:
            self.get_logger().info("[FullCourse] no data")
            return
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"[FullCourse] SUMMARY checkpoints={self.checkpoints} failed={self.failed}"
        )
        self.get_logger().info(f"  overall: {self._phase_stats(self.heights, 'height')}")
        self.get_logger().info(f"  overall: max|pitch|={max(abs(p) for p in self.pitches):.3f} "
                               f"max|roll|={max(abs(r) for r in self.rolls):.3f}")
        for k, v in self.heights_by_phase.items():
            self.get_logger().info(f"  phase: {self._phase_stats(v, k)}")
        self.get_logger().info(f"  final x={self.xs[-1]:.2f} y={self.ys[-1]:.2f} z={self.heights[-1]:.3f}")
        self.get_logger().info("=" * 70)


def main():
    rclpy.init()
    node = FullCourseTestNode()
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
