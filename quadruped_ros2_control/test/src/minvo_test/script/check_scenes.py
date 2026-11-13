#!/usr/bin/env python3
"""Check exported production trajectories; use nominal boxes only for collision witnesses."""
import csv
import math
from pathlib import Path

import yaml


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    data = Path(__file__).resolve().parents[1] / 'data'
    summary = []
    for name in ['single', 'pillars', 'stairs']:
        config = yaml.safe_load((data / name / 'config.yaml').read_text())
        curves = read(data / name / 'trajectories.csv')
        metrics = read(data / name / 'metrics.csv')
        smoothness = read(data / name / 'smoothness.csv')
        radius = config['foot_radius']
        assert all(int(r['valid']) == 1 and int(r['certificate']) == 0 for r in metrics), name
        assert all(float(r['endpoint_error']) < 1e-8 for r in metrics), name
        assert len(smoothness) == len(metrics) + len(config['swings']), name
        for row in smoothness:
            assert int(row['min_internal_continuity']) >= 2, (name, row)
            assert float(row['max_position_gap']) < 1e-6, (name, row)
            assert float(row['max_velocity_gap']) < 1e-5, (name, row)
            assert float(row['max_acceleration_gap']) < 1e-3, (name, row)
            assert abs(float(row['lift_off_vz']) - config['reference']['lift_off_velocity']) < 1e-8
            assert abs(float(row['touch_down_vz']) - config['reference']['touch_down_velocity']) < 1e-8
            assert float(row['lift_off_acceleration']) < 1e-7
            assert float(row['touch_down_acceleration']) < 1e-7
        nominal_margin = 0.0
        endpoint_error = 0.0
        max_speed = 0.0
        max_acceleration = 0.0
        for index, swing in enumerate(config['swings']):
            for method in {r['method'] for r in curves}:
                group = [r for r in curves if int(r['swing']) == index and r['method'] == method]
                for contact in [group[0], group[-1]]:
                    endpoint_error = max(endpoint_error, abs(float(contact['z']) - float(contact['surface_z'])))
                if method != 'reference':
                    continue
                max_speed = max(max_speed, max(abs(float(r['vz'])) for r in group))
                max_acceleration = max(max_acceleration, max(abs(float(r['az'])) for r in group))
                ref = config['reference']
                mid = min(group, key=lambda r: abs(float(r['t']) - swing['duration'] / 2))
                expected = max(float(group[0]['z']), float(group[-1]['z'])) + min(
                    1, swing['duration'] / ref['swing_time_scale']) * ref['swing_height']
                assert abs(float(mid['z']) - expected) < 1e-8, name
                # Exact sphere-to-nominal-solid-box distance at exported sample times.
                # A negative witness proves intersection; sampling does not certify safety.
                for row in group:
                    x, y, z = (float(row[k]) for k in ['x', 'y', 'z'])
                    for x0, x1, y0, y1, height in config['obstacles']:
                        distance = math.sqrt(max(x0-x, 0, x-x1)**2 + max(y0-y, 0, y-y1)**2 +
                                             max(z+radius-height, 0)**2)
                        nominal_margin = min(nominal_margin, distance - radius)
        assert endpoint_error < 1e-8, name
        assert nominal_margin < -1e-4, f'{name}: missing reference collision witness'
        if name == 'pillars':
            assert len(config['swings']) == len(config['obstacles']) == 4
            for swing, box in zip(config['swings'], config['obstacles']):
                x, y, _ = swing['goal']
                assert box[0] + radius < x < box[1] - radius
                assert box[2] + radius < y < box[3] - radius
        assert all(float(r['crossing_violation']) < 1e-8 for r in metrics
                   if r['method'] != 'MINVO_geometry'), name
        summary.append(dict(scene=name, valid_plans=len(metrics), endpoint_error_m=endpoint_error,
                            reference_nominal_sphere_margin_m=nominal_margin,
                            reference_grid_clearance_m=min(float(r['reference_clearance']) for r in metrics),
                            max_reference_abs_vz_mps=max_speed,
                            max_reference_abs_az_mps2=max_acceleration,
                            max_optimized_speed_mps=max(float(r['max_speed']) for r in smoothness
                                                        if r['method'] != 'reference'),
                            max_optimized_acceleration_mps2=max(float(r['max_acceleration']) for r in smoothness
                                                               if r['method'] != 'reference')))
        print(summary[-1])
    with (data / 'validation.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=summary[0].keys())
        writer.writeheader()
        writer.writerows(summary)


if __name__ == '__main__':
    main()
