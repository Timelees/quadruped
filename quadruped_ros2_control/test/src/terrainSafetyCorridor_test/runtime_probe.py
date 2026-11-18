#!/usr/bin/env python3
"""Short Gazebo-only smoke test: stand, trot, optional forward, stop; collect actual topics."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from grid_map_msgs.msg import GridMap

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--seconds', type=float, default=25)
parser.add_argument('--forward', type=float, default=0.0)
parser.add_argument('--baseline', action='store_true', help='Expect corridor mode to be disabled')
args = parser.parse_args()
rclpy.init()
node = Node('terrain_corridor_runtime_probe')
publisher = node.create_publisher(Inputs, '/control_input', 10)
rows = []
counts = {'corridor_messages': 0, 'nonempty_corridor_messages': 0, 'map_messages': 0}
start = time.monotonic()
failed = False

def odometry(msg):
    global failed
    q = msg.pose.pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
    p = msg.pose.pose.position
    rows.append([time.monotonic() - start, p.x, p.y, p.z, roll, pitch])
    if time.monotonic() - start > 5 and (p.z < 0.1 or abs(roll) > 0.78 or abs(pitch) > 0.78):
        failed = True

def corridors(msg):
    counts['corridor_messages'] += 1
    counts['nonempty_corridor_messages'] += int(any(m.action == 0 and len(m.points) > 0 for m in msg.markers))

def elevation(msg):
    counts['map_messages'] += 1
    counts['map_layers'] = list(msg.layers)

node.create_subscription(Odometry, '/odom', odometry, 10)
node.create_subscription(MarkerArray, '/terrain_safety_corridor', corridors, 1)
node.create_subscription(GridMap, '/elevation_map_raw', elevation, 1)

def send(command, forward=0.0):
    message = Inputs()
    message.command = command
    message.ly = forward
    publisher.publish(message)

try:
    while rclpy.ok() and time.monotonic() - start < args.seconds and not failed:
        t = time.monotonic() - start
        # Keep startup commands long enough for DDS discovery; later command=0 preserves the selected gait.
        command = 2 if t < 2.0 else (3 if 3.0 < t < 4.0 else 0)
        send(command, args.forward if 6 < t < args.seconds - 3 else 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
finally:
    # Leave the simulated robot standing even on a failed probe.
    stop = time.monotonic() + 1
    while rclpy.ok() and time.monotonic() < stop:
        send(1 if failed else (2 if time.monotonic() < stop - 0.7 else 0))
        rclpy.spin_once(node, timeout_sec=0.02)
    result = dict(counts, fell=failed, odometry_samples=len(rows), last_odometry=rows[-1] if rows else None,
                  max_abs_roll=max((abs(r[4]) for r in rows), default=None),
                  max_abs_pitch=max((abs(r[5]) for r in rows), default=None))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    args.output.with_suffix('.csv').write_text('wall_seconds,x,y,z,roll,pitch\n' + '\n'.join(','.join(map(str, row)) for row in rows) + '\n')
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
if failed or not rows or (not args.baseline and counts['nonempty_corridor_messages'] == 0) or counts['map_messages'] == 0:
    raise SystemExit(1)
