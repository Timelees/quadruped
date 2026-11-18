#!/usr/bin/env python3
"""Drive one independently spawned stair10cm trial; retain odometry and WBC samples.

The platform trial starts at the origin, selects trot, turns +90 degrees using rx,
then approaches y=4. Success requires all four feet on the destination and a hold.
Inputs are normalized; --speed .5 requests .175 m/s with reference.info defaults.
"""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from control_input_msgs.msg import Inputs
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray
from grid_map_msgs.msg import GridMap


def clamp(x, limit):
    return max(-limit, min(limit, x))


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', choices=['stairs', 'platform'], required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--speed', type=float, default=.5)
    p.add_argument('--sim-seconds', type=float, default=100)
    p.add_argument('--wall-seconds', type=float, default=600)
    p.add_argument('--target', type=float, help='Base target: default stairs x=4.95, platform y=4.0')
    args = p.parse_args()
    rclpy.init()
    node = Node('terrain_scenario_probe')
    pub = node.create_publisher(Inputs, '/control_input', 10)
    rows, wbc, references, estimates, map_samples = [], [], [], [], []
    phase = 'initialize'
    start_sim = None
    latest = None
    start_wall = time.monotonic()
    phase_start = 0.0
    hold_since = None
    turn_since = None
    result = 'timeout'
    counts = {'map': 0, 'corridor': 0}

    def odom(m):
        nonlocal start_sim, latest
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if start_sim is None:
            start_sim = t
        q, pos, vel = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(clamp(2*(q.w*q.y-q.z*q.x), 1))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        latest = [t, pos.x, pos.y, pos.z, roll, pitch, yaw, vel.linear.x, vel.linear.y,
                  vel.linear.z, vel.angular.x, vel.angular.y, vel.angular.z]
        rows.append(latest + [phase])

    def diag(m):
        if len(m.data) in (78, 83):
            wbc.append(list(m.data))

    def reference(m):
        references.extend([list(m.data[i:i+25]) for i in range(0, len(m.data), 25)])

    def maps(m):
        counts['map'] += 1
        if counts['map'] % 10 or 'elevation' not in m.layers:
            return
        res, info = m.info.resolution, m.info
        nx, ny = round(info.length_x/res), round(info.length_y/res)
        data = m.data[m.layers.index('elevation')]
        column_major = data.layout.dim[0].label == 'column_index'
        sites = [(1.8,0.,0.)]+[(2.4+.3*i,0.,.1*(i+1)) for i in range(7)]+[(0.,4.,.125)]
        for site, (px,py,height) in enumerate(sites):
            values = []
            for dx in [-.02,0.,.02]:
                for dy in [-.02,0.,.02]:
                    i = math.floor((info.pose.position.x+info.length_x/2-px-dx)/res)
                    j = math.floor((info.pose.position.y+info.length_y/2-py-dy)/res)
                    if 0 <= i < nx and 0 <= j < ny:
                        i, j = (i+m.outer_start_index)%nx, (j+m.inner_start_index)%ny
                        v = data.data[i+nx*j if column_major else j+ny*i]
                        if math.isfinite(v): values.append(v)
            if values:
                stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                map_samples.append([stamp,site,px,py,height,statistics.median(values),min(values),max(values),len(values)])

    def markers(m):
        counts['corridor'] += int(any(x.action == 0 and x.points for x in m.markers))

    node.create_subscription(Odometry, '/Ground_Truth', odom, 30)
    def estimate(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        estimates.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9, p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    node.create_subscription(Odometry, '/odom', estimate, 30)
    node.create_subscription(Float64MultiArray, '/wbc_diagnostics', diag, 100)
    node.create_subscription(GridMap, '/elevation_map_raw', maps, 1)
    node.create_subscription(Float64MultiArray, '/swing_reference', reference, 10)
    node.create_subscription(MarkerArray, '/terrain_safety_corridor', markers, 1)
    last_report = -10
    try:
        while rclpy.ok() and time.monotonic() - start_wall < args.wall_seconds:
            rclpy.spin_once(node, timeout_sec=.01)
            msg = Inputs()
            msg.command = 2 if phase == 'stand' else (3 if phase == 'trot' else 0)
            if latest is None:
                pub.publish(msg)
                continue
            t = latest[0] - start_sim
            x, y, z, roll, pitch, yaw = latest[1:7]
            if t > args.sim_seconds:
                break
            if t > 7 and (abs(roll) > .78 or abs(pitch) > .85 or z < .10):
                result = 'fell'
                break
            if wbc and wbc[-1][5] == 0:
                result = 'wbc_invalid'
                break
            if phase == 'initialize' and t > 2:
                phase, phase_start = 'stand', t
            elif phase == 'stand' and t - phase_start > 3:
                phase, phase_start = 'trot', t
            elif phase == 'trot' and t - phase_start > 2:
                phase, phase_start = ('turn' if args.scenario == 'platform' else 'forward'), t
            elif phase == 'turn':
                error = angle(math.pi/2 - yaw)
                msg.rx = -clamp(.9*error, .35)
                if abs(error) < .035:
                    turn_since = t if turn_since is None else turn_since
                    if t - turn_since > .7:
                        phase, phase_start = 'settle_turn', t
                else:
                    turn_since = None
            elif phase == 'settle_turn' and t - phase_start > 1:
                phase, phase_start = 'forward', t
            elif phase == 'forward':
                target = args.target if args.target is not None else (4.95 if args.scenario == 'stairs' else 4.0)
                remaining = target - (x if args.scenario == 'stairs' else y)
                # Keep the approach above the controller's low-speed drift; stop at the target.
                msg.ly = min(args.speed, max(.25, remaining*2)) if remaining >= .025 else 0.
                desired_yaw = 0 if args.scenario == 'stairs' else math.pi/2
                msg.rx = -clamp(.6*angle(desired_yaw-yaw), .15)
                # Small cross-track correction, using the existing body-frame velocity input.
                msg.lx = clamp((y if args.scenario == 'stairs' else x)*(.7 if args.scenario == 'stairs' else -.7), .15)
                if remaining < .025:
                    phase, phase_start = 'hold', t
            elif phase == 'hold':
                # Continue trot without translation: success includes three seconds of support stability.
                feet = [wbc[-1][30+12*i:33+12*i] for i in range(4)] if wbc else []
                on_destination = len(feet) == 4 and all(
                    (4.35 < f[0] < 6.35 and abs(f[1]) < 1.45 and f[2] > .69)
                    if args.scenario == 'stairs' else
                    (abs(f[0]) < .98 and 3.5 < f[1] < 4.5 and f[2] > .115) for f in feet)
                stable = on_destination and abs(roll) < .2 and abs(pitch) < .25
                hold_since = (t if hold_since is None else hold_since) if stable else None
                if hold_since is not None and t - hold_since > 3:
                    result = 'passed'
                    break
                if t - phase_start > 12:
                    result = 'destination_not_stable'
                    break
            pub.publish(msg)
            if t - last_report >= 5:
                last_report = t
                print(f't={t:.1f} phase={phase} xyz=({x:.3f},{y:.3f},{z:.3f}) rpy=({roll:.3f},{pitch:.3f},{yaw:.3f}) wbc={len(wbc)}', flush=True)
    except Exception:
        result = 'probe_error'
        raise
    finally:
        msg = Inputs()
        msg.command = 1
        pub.publish(msg)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        header = ['sim_time','x','y','z','roll','pitch','yaw','vx','vy','vz','wx','wy','wz','phase']
        with args.output.with_suffix('.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(header); writer.writerows(rows)
        with args.output.with_name(args.output.stem+'_estimate.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(['ros_time','x','y','z','qx','qy','qz','qw']); writer.writerows(estimates)
        with args.output.with_name(args.output.stem+'_map.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(['ros_time','site','x','y','true_surface','median_elevation','min_elevation','max_elevation','cells']); writer.writerows(map_samples)
        names = ['sim_time','mode','status','nwsr','constraint_violation','valid']
        names += [f'tau_{i}' for i in range(12)] + [f'force_{i}' for i in range(12)]
        names += [f'{leg}_{q}_{axis}' for leg in ['LF','RF','LH','RH']
                  for q in ['p','v','pd','vd'] for axis in 'xyz']
        if wbc and len(wbc[0]) == 83:
            names += ['ros_time','sensor_LF','sensor_RF','sensor_LH','sensor_RH']
        with args.output.with_name(args.output.stem+'_wbc.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(names); writer.writerows(wbc)
        with args.output.with_name(args.output.stem+'_reference.csv').open('w') as f:
            writer = csv.writer(f)
            writer.writerow(['sim_time']+[f'{leg}_{q}_{axis}' for leg in ['LF','RF','LH','RH'] for q in ['p','v'] for axis in 'xyz'])
            writer.writerows(references)
        summary = dict(result=result, scenario=args.scenario, speed_normalized=args.speed,
                       wall_seconds=time.monotonic()-start_wall, samples=len(rows), wbc_samples=len(wbc),
                       final=latest, counts=counts,
                       max_abs_roll=max((abs(r[4]) for r in rows), default=None),
                       max_abs_pitch=max((abs(r[5]) for r in rows), default=None))
        args.output.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if result == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
