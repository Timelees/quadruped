#!/usr/bin/env python3
"""Compare Gazebo foot spheres with the trial's actual static world geometry.

Pose_V link poses are model-relative in this world. Compose them with the robot
pose before querying geometry. These are sampled observations, not certificates.
"""
import argparse
import csv
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

LEGS = ('LF', 'RF', 'LH', 'RH')
LINKS = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')


def rotation(q):
    x, y, z, w = q
    if abs(np.dot(q, q)-1) > 1e-4:
        raise ValueError('Nonunit pose quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform(element):
    text = element.findtext('pose', '0 0 0 0 0 0')
    x, y, z, roll, pitch, yaw = map(float, text.split())
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def world_boxes(path):
    boxes = []
    for model in ET.parse(path).getroot().findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size:
                    pose = transform(model) @ transform(link) @ transform(collision)
                    boxes.append((pose[:3, 3], pose[:3, :3], np.array(list(map(float, size.split())))/2))
                elif collision.find('geometry/plane') is not None:
                    if model.get('name') != 'ground_plane':
                        raise ValueError('Unexpected plane: only the z=0 ground is supported')
                else:
                    raise ValueError('Unsupported world collision geometry')
    return boxes


def signed_clearance(points, boxes, radius=.019):
    result = points[:, 2]-radius
    for centre, orientation, half in boxes:
        delta = np.abs((points-centre) @ orientation)-half
        distance = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(delta.max(axis=1), 0)
        result = np.minimum(result, distance-radius)
    return result


def true_feet(path):
    times, positions = [], []
    with path.open() as stream:
        for stamp, group in itertools.groupby(csv.DictReader(stream), key=lambda row: float(row['sim_time'])):
            poses = {row['name']: row for row in group if row['name'] in ('robot',)+LINKS}
            if len(poses) != 5:
                continue
            base = poses['robot']
            orientation = rotation(np.array([float(base[k]) for k in ('qx', 'qy', 'qz', 'qw')]))
            translation = np.array([float(base[k]) for k in 'xyz'])
            positions.append([orientation @ np.array([float(poses[link][k]) for k in 'xyz'])+translation for link in LINKS])
            times.append(stamp)
    if not times or np.any(np.diff(times) <= 0):
        raise ValueError('Missing or nonmonotone Gazebo poses')
    return np.array(times), np.array(positions)


def analyze(trial, default_world):
    prefix = trial.with_suffix('')
    times, positions = true_feet(Path(str(prefix)+'_gz_poses.csv'))
    world = trial.with_suffix('.world.sdf')
    boxes = world_boxes(world if world.exists() else default_world)
    with Path(str(prefix)+'_wbc.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    stamp = np.array([float(row['ros_time']) for row in rows])
    if np.any(np.diff(stamp) <= 0):
        raise ValueError('Nonmonotone WBC ROS timestamps')
    index = np.searchsorted(stamp, times, side='right')-1
    index = np.clip(index, 0, len(rows)-1)
    matched = (times >= stamp[0]) & (times-stamp[index] < .0075)
    summary = {'definition': 'Signed distance from true 19 mm foot spheres to actual world plane/boxes; model-relative poses composed with robot pose; last WBC ROS sample within 7.5 ms',
               'pose_frames': len(times), 'matched_frames': int(matched.sum()), 'legs': {}}
    output = []
    for leg_index, (leg, bit) in enumerate(zip(LEGS, (8, 4, 2, 1))):
        clearance = signed_clearance(positions[:, leg_index], boxes)
        mode = np.array([int(float(rows[i]['mode'])) for i in index])
        status = np.array([int(float(rows[i][leg+'_safety_status'])) for i in index])
        swing = matched & ((mode & bit) == 0)
        active = swing & (status == 1)  # LandingSolved is outside the noncontact interval.
        metrics = {}
        for name, mask in (('scheduled_swing', swing), ('active_noncontact', active)):
            values = clearance[mask]*1000
            metrics[name] = dict(samples=len(values), minimum_mm=float(values.min()) if len(values) else None,
                p05_mm=float(np.percentile(values, 5)) if len(values) else None,
                penetration_below_minus_1mm_samples=int(np.sum(values < -1)))
        summary['legs'][leg] = metrics
        for i in np.flatnonzero(swing):
            output.append((times[i], leg, *positions[i, leg_index], clearance[i], int(status[i])))
    with trial.with_suffix('.true_clearance.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['ros_time', 'leg', 'x', 'y', 'z', 'signed_clearance_m', 'safety_status'])
        writer.writerows(output)
    trial.with_suffix('.true_geometry.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs='+')
    parser.add_argument('--world', type=Path, required=True)
    args = parser.parse_args()
    for trial in args.trials:
        print(trial.name, json.dumps(analyze(trial, args.world)['legs']))


if __name__ == '__main__':
    main()
