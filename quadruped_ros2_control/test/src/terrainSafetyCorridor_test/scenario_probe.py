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
    p.add_argument('--scenario', choices=['trot', 'stairs', 'platform', 'obstacle', 'trench', 'piles'], required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--speed', type=float, default=.5)
    p.add_argument('--sim-seconds', type=float, default=100)
    p.add_argument('--wall-seconds', type=float, default=600)
    p.add_argument('--target', type=float, help='Base target: stairs x=4.95, platform y=4.0; obstacle uses the far side')
    p.add_argument('--record-gz-poses', action='store_true')
    p.add_argument('--wrench-force', type=float, default=0.)
    p.add_argument('--wrench-duration', type=float, default=.06)
    args = p.parse_args()
    longitudinal = args.scenario in ('stairs', 'obstacle')
    targets = {'stairs': 4.95, 'platform': 4.0, 'obstacle': -.35, 'trench': 5.85, 'piles': 8.15}
    obstacle_direction = None
    scan_yaw = None
    scan_angle = 0.0
    scan_complete = False
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
        if len(m.data) in (78, 83, 112, 113, 151, 163):
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
    gz_poses, wrench_events = [], []
    wrench_link_id = None
    wrench_start = None
    wrench_finished = False
    transport = None
    if args.record_gz_poses or args.wrench_force:
        from gz.transport13 import Node as TransportNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        from gz.msgs10.entity_pb2 import Entity
        transport = TransportNode()
        def poses(message):
            nonlocal wrench_link_id
            stamp = message.header.stamp.sec + message.header.stamp.nsec*1e-9
            for pose in message.pose:
                if any(token in pose.name for token in ('robot', 'body', 'FL_', 'FR_', 'RL_', 'RR_')):
                    if pose.name == 'FL_knee':
                        wrench_link_id = pose.id
                    position, orientation = pose.position, pose.orientation
                    parent = '|'.join(','.join(item.value) for item in pose.header.data if item.key == 'frame_id')
                    gz_poses.append([stamp,pose.id,pose.name,parent,position.x,position.y,position.z,
                                     orientation.x,orientation.y,orientation.z,orientation.w])
        if args.record_gz_poses:
            transport.subscribe(Pose_V, '/world/stair10cm/dynamic_pose/info', poses)
        if args.wrench_force:
            wrench_pub = transport.advertise('/world/stair10cm/wrench/persistent', EntityWrench)
            clear_pub = transport.advertise('/world/stair10cm/wrench/clear', Entity)
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
            if args.scenario == 'obstacle' and obstacle_direction is None:
                obstacle_direction = 1 if x < -1.2 else -1
                targets['obstacle'] = -.35 if obstacle_direction == 1 else -2.05
            heading = (0.0 if args.scenario == 'stairs' or obstacle_direction == 1 else
                       math.pi if args.scenario == 'obstacle' else math.pi/2)
            if args.wrench_force and not wrench_finished and t > 12 and wbc:
                # Trigger once during an observed LF swing away from the ground.
                lf_swing = not (int(wbc[-1][1]) & 8) and wbc[-1][32] > .06
                if wrench_start is None and lf_swing and wrench_link_id is not None:
                    pulse = EntityWrench()
                    pulse.entity.name = 'robot::FL_knee'; pulse.entity.type = Entity.LINK
                    pulse.entity.id = wrench_link_id
                    pulse.wrench.force.z = -args.wrench_force
                    ok = wrench_pub.publish(pulse)
                    wrench_start = latest[0]
                    wrench_events.append(dict(event='start',sim_time=wrench_start,force_z_N=-args.wrench_force,
                                              entity_id=wrench_link_id,published=ok))
                elif wrench_start is not None and latest[0]-wrench_start >= args.wrench_duration:
                    entity = Entity(); entity.name = 'robot::FL_knee'; entity.type = Entity.LINK
                    entity.id = wrench_link_id
                    ok = clear_pub.publish(entity)
                    wrench_events.append(dict(event='clear',sim_time=latest[0],published=ok,
                                              observed_duration_s=latest[0]-wrench_start))
                    wrench_finished = True
            if t > args.sim_seconds:
                stable_trot = hold_since is not None and t - hold_since >= 3 and any(
                    row[1] in (6, 9) and row[5] == 1 for row in wbc)
                if args.scenario == 'trot' and stable_trot:
                    result = 'passed'
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
            elif phase == 'trot' and args.scenario == 'trot':
                upright = abs(roll) < .2 and abs(pitch) < .25 and z > .17
                hold_since = (t if hold_since is None else hold_since) if upright else None
            elif phase == 'trot' and t - phase_start > 2:
                phase, phase_start = ('forward' if args.scenario == 'stairs' or obstacle_direction == 1 else 'turn'), t
            elif phase == 'turn':
                error = angle(heading - yaw)
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
                target = args.target if args.target is not None else targets[args.scenario]
                direction = obstacle_direction if args.scenario == 'obstacle' else 1
                remaining = direction * (target - (x if longitudinal else y))
                # Keep the approach above the controller's low-speed drift; stop at the target.
                msg.ly = min(args.speed, max(.25, remaining*2)) if remaining >= .025 else 0.
                msg.rx = -clamp(.6*angle(heading-yaw), .15)
                # Small cross-track correction, using the existing body-frame velocity input.
                msg.lx = clamp((direction*y if longitudinal else x)*(.7 if longitudinal else -.7), .15)
                if args.scenario in ('trench', 'piles') and not scan_complete and 4.0 <= y < 4.35:
                    phase, phase_start = 'scan', t
                    scan_yaw = yaw
                elif remaining < .025:
                    phase, phase_start = 'hold', t
            elif phase == 'scan':
                # Reproduce the established mapping prerequisite on the platform:
                # a complete turn before approaching the trench, then face +Y.
                scan_angle += angle(yaw - scan_yaw)
                scan_yaw = yaw
                msg.rx = -.4
                if scan_angle >= 2*math.pi-.08:
                    scan_complete = True
                    phase, phase_start = 'turn', t
                    turn_since = None
            elif phase == 'hold':
                # Continue trot without translation: success includes three seconds of support stability.
                feet = [wbc[-1][30+12*i:33+12*i] for i in range(4)] if wbc else []
                def on_destination(f):
                    if args.scenario == 'stairs':
                        return 4.35 < f[0] < 6.35 and abs(f[1]) < 1.45 and f[2] > .69
                    if args.scenario == 'obstacle':
                        lo, hi = (-.95, .3) if obstacle_direction == 1 else (-2.7, -1.45)
                        return lo < f[0] < hi and abs(f[1]) < .5 and f[2] > -.02
                    lo, hi = {'trench': (5.35, 6.35), 'piles': (7.65, 8.65)}.get(args.scenario, (3.5, 4.5))
                    return abs(f[0]) < .98 and lo < f[1] < hi and f[2] > .115
                on_destination = len(feet) == 4 and all(on_destination(f) for f in feet)
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
    except KeyboardInterrupt:
        result = 'interrupted'
    except Exception:
        result = 'probe_error'
        raise
    finally:
        # Stop native callbacks before freezing CSV data and before interpreter
        # shutdown; otherwise Gazebo's transport thread can outlive Python.
        if transport is not None and args.record_gz_poses:
            transport.unsubscribe('/world/stair10cm/dynamic_pose/info')
        msg = Inputs()
        msg.command = 1
        if rclpy.ok():
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
        if wbc and len(wbc[0]) >= 83:
            names += ['ros_time','sensor_LF','sensor_RF','sensor_LH','sensor_RH']
        if wbc and len(wbc[0]) >= 112:
            names += ['control_period'] + [f'{leg}_{field}' for leg in ['LF','RF','LH','RH']
                for field in ['safety_status','clearance','psi1','tube_slack','correction','wbc_safety_residual','tube_status']]
        if wbc and len(wbc[0]) >= 113:
            names.append('wbc_compute_ms')
        if wbc and len(wbc[0]) >= 151:
            names += [f'{leg}_{quantity}_{axis}' for quantity in ['command_accel','reference_accel','nominal_accel']
                      for leg in ['LF','RF','LH','RH'] for axis in 'xyz']
            names += ['policy_accepted','policy_rejected']
        if wbc and len(wbc[0]) >= 163:
            names += [f'{leg}_baseline_accel_{axis}' for leg in ['LF','RF','LH','RH'] for axis in 'xyz']
        with args.output.with_name(args.output.stem+'_wbc.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(names); writer.writerows(wbc)
        with args.output.with_name(args.output.stem+'_reference.csv').open('w') as f:
            writer = csv.writer(f)
            writer.writerow(['sim_time']+[f'{leg}_{q}_{axis}' for leg in ['LF','RF','LH','RH'] for q in ['p','v'] for axis in 'xyz'])
            writer.writerows(references)
        if args.wrench_force and wrench_start is not None and not wrench_finished:
            entity = Entity(); entity.name='robot::FL_knee'; entity.type=Entity.LINK
            entity.id = wrench_link_id
            clear_pub.publish(entity)
            wrench_events.append(dict(event='cleanup_clear',sim_time=latest[0] if latest else None))
        args.output.with_suffix('.wrench.json').write_text(json.dumps(wrench_events,indent=2)+'\n')
        if args.record_gz_poses:
            with args.output.with_name(args.output.stem+'_gz_poses.csv').open('w') as stream:
                writer=csv.writer(stream)
                writer.writerow(['sim_time','id','name','parent_frame','x','y','z','qx','qy','qz','qw'])
                writer.writerows(gz_poses)
        summary = dict(result=result, scenario=args.scenario, speed_normalized=args.speed,
                       obstacle_direction=obstacle_direction,
                       wall_seconds=time.monotonic()-start_wall, samples=len(rows), wbc_samples=len(wbc),
                       final=latest, counts=counts,
                       max_abs_roll=max((abs(r[4]) for r in rows), default=None),
                       max_abs_pitch=max((abs(r[5]) for r in rows), default=None))
        args.output.with_suffix('.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        transport = None
    return 0 if result == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
