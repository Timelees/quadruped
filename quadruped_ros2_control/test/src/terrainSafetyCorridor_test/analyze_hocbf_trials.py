#!/usr/bin/env python3
"""Summarize explicit independent trials; retain failures and conditional coverage."""
import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from analyze_true_geometry import true_feet, world_boxes

LEGS = ('LF', 'RF', 'LH', 'RH')
GEOMETRY = (1, 13, 15, 16)
CONTRACT = (1, 13)


def wilson(passed, total):
    if not total:
        return None
    z = 1.959963984540054
    rate = passed / total
    scale = 1 + z*z/total
    centre = (rate+z*z/(2*total))/scale
    radius = z*math.sqrt(rate*(1-rate)/total+z*z/(4*total*total))/scale
    return [max(0., centre-radius), min(1., centre+radius)]


def quantiles(values):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    return dict(n=len(finite), P50_P95_P99=np.percentile(finite, [50, 95, 99]).tolist() if len(finite) else None,
                maximum=float(finite.max()) if len(finite) else None)



def terrain_height(points, boxes):
    """Exact vertical ray/box intersections, including rotated static boxes."""
    top = np.zeros(len(points))
    for centre, orientation, half in boxes:
        origin = (np.column_stack((points, np.zeros(len(points))))-centre) @ orientation
        direction = np.array([0.,0.,1.]) @ orientation
        lower, upper = np.full(len(points), -np.inf), np.full(len(points), np.inf)
        valid = np.ones(len(points), dtype=bool)
        for axis in range(3):
            if abs(direction[axis]) < 1e-12:
                valid &= np.abs(origin[:,axis]) <= half[axis]
            else:
                a = (-half[axis]-origin[:,axis])/direction[axis]
                b = (half[axis]-origin[:,axis])/direction[axis]
                lower = np.maximum(lower,np.minimum(a,b));upper=np.minimum(upper,np.maximum(a,b))
        valid &= lower <= upper
        top = np.maximum(top,np.where(valid,upper,-np.inf))
    return top


def map_evidence(path):
    source = path.with_suffix('.certified_map.csv')
    if not source.is_file():
        return None
    with source.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return dict(samples=0)
    xy = np.array([[float(r['x']),float(r['y'])] for r in rows])
    certified = np.array([float(r['certified_height_m']) for r in rows])
    boxes = world_boxes(path.with_suffix('.world.sdf') if path.with_suffix('.world.sdf').exists() else
        Path(__file__).resolve().parents[3]/'libraries/gz_quadruped_playground/worlds/stair10cm.sdf')
    truth = terrain_height(xy,boxes)
    known = np.isfinite(certified)
    error = truth[known]-certified[known]
    return dict(samples=len(rows),known=int(known.sum()),unknown=int((~known).sum()),
                true_minus_certified_height_m=quantiles(error),
                underestimation_above_5mm_samples=int(np.sum(error>.005)),
                definition='Spatial samples (new trials: 2 cm near obstacles and 10 cm elsewhere; old trials: 10 cm), at most 1 map per simulated second; unknowns retained; static SDF surface at map XY with nominal map/world registration; includes perception and registration error')


def obstacle_crossings(path,scenario):
    if scenario not in ('obstacle','course'):
        return None
    stamps,feet = true_feet(path.with_name(path.stem+'_gz_poses.csv'))
    if scenario == 'obstacle':
        manifest=json.loads(path.with_suffix('.manifest.json').read_text())
        obstacle=manifest['trial.obstacle'];centres=[obstacle['pose'][0]]
        height=[obstacle['pose'][2]+.5*obstacle['collision_box_size'][2]]
        lane=0.; half_width=.5*obstacle['collision_box_size'][1];direction=-1 if feet[0,:,0].mean()>centres[0] else 1
    else:
        manifest=json.loads(path.with_suffix('.manifest.json').read_text())
        bars=manifest.get('trial.controlled_bars',[(1.1,.06,.04),(1.8,.08,.07),(2.5,.12,.05),(3.2,.06,.09)])
        centres=[b[0] for b in bars];height=[b[2] for b in bars];lane=-3.;half_width=.7;direction=1
    crossings=[]
    for centre,h in zip(centres,height):
        for leg_index,leg in enumerate(LEGS):
            x=feet[:,leg_index,0]
            indices=np.flatnonzero((direction*(x[:-1]-centre)<0)&(direction*(x[1:]-centre)>=0))
            if not len(indices):
                crossings.append(dict(leg=leg,obstacle_x=centre,observed=False));continue
            k=int(indices[0]);weight=(centre-x[k])/(x[k+1]-x[k]);point=feet[k,leg_index]+weight*(feet[k+1,leg_index]-feet[k,leg_index])
            crossings.append(dict(leg=leg,obstacle_x=centre,observed=True,pose_gap_s=float(stamps[k+1]-stamps[k]),
                y_m=float(point[1]),within_span=bool(abs(point[1]-lane)<=half_width-.019),
                top_clearance_m=float(point[2]-h-.019)))
    return dict(crossings=crossings,all_feet_crossed_within_span=all(r.get('within_span',False) for r in crossings),
                definition='First true foot-centre crossing of each obstacle centre plane, interpolated between native poses; sphere must fit within obstacle lateral span')


def analyze(path):
    outcome = json.loads(path.read_text())
    manifest = json.loads(path.with_suffix('.manifest.json').read_text())
    with path.with_name(path.stem+'_wbc.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    result = dict(trial=path.stem, result=outcome['result'], scenario=outcome['scenario'],
                  mode=outcome['safety_filter'], online_replan=outcome['online_replan'],
                  wrench=manifest.get('trial.wrench'), replan_applied=outcome['replan_applied'],
                  world_hash=manifest['trial.world.sdf'], binary_hash=manifest.get(
                      'install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so'),
                  received_wbc_samples=len(rows))
    result['seed'] = manifest.get('trial.seed')
    pulse_path = path.with_suffix('.wrench.json')
    events = json.loads(pulse_path.read_text()) if pulse_path.exists() else []
    result['pulse_started'] = any(event['event'] == 'start' for event in events)
    result['pulse_observed_duration_s'] = next((event['observed_duration_s'] for event in events
                                               if 'observed_duration_s' in event), None)
    protocol = dict(world=result['world_hash'], wrench=result['wrench'], spawn=manifest.get('trial.spawn'),
                    task=manifest['trial.task.info'], speed=outcome['speed_normalized'])
    result['protocol_hash'] = hashlib.sha256(json.dumps(protocol,sort_keys=True).encode()).hexdigest()
    statuses = Counter()
    swing_count = geometry = contract = corrected = 0
    deviations, corrections, violations, residuals, delay, pos, vel = [], [], [], [], [], [], []
    all_swing_deviations = []
    for leg, bit in zip(LEGS, (8, 4, 2, 1)):
        for index, row in enumerate(rows):
            previous = rows[index-1] if index else row
            status = int(float(row[leg+'_safety_status']))
            swing = not (int(float(row['mode'])) & bit)
            if not swing:
                continue
            swing_count += 1
            statuses[status] += 1
            deviation = max(abs(float(row[f'{leg}_command_accel_{a}'])-
                                float(row[f'{leg}_baseline_accel_{a}'])) for a in 'xyz')
            all_swing_deviations.append(deviation)
            if status in GEOMETRY:
                geometry += 1
                contract += result['mode'] == 'full' and status in CONTRACT
                deviations.append(deviation)
                correction = float(row[leg+'_correction'])
                corrections.append(correction)
                corrected += deviation > 1e-6
                violations.append(max(0., -float(row[leg+'_wbc_safety_residual'])))
                dt = float(row['sim_time'])-float(previous['sim_time'])
                if (int(float(previous[leg+'_safety_status'])) in GEOMETRY and
                    not (int(float(previous['mode'])) & bit) and .003 < dt < .0075 and
                    float(row['sensor_'+leg]) < 3 and float(previous['sensor_'+leg]) < 3):
                    residuals.append(math.sqrt(sum(((float(row[f'{leg}_v_{a}'])-
                        float(previous[f'{leg}_v_{a}']))/dt-float(previous[f'{leg}_command_accel_{a}']))**2 for a in 'xyz')))
    replans = json.loads(path.with_suffix('.replans.json').read_text())
    for applied in replans['applied']:
        delay.append(1000*(applied['applied']-applied['activation']))
        pos.append(1000*applied['position_error'])
        vel.append(applied['velocity_error'])
    policy_times = [float(row['sim_time']) for previous, row in zip(rows, rows[1:])
                    if float(row['policy_accepted']) > float(previous['policy_accepted'])]
    envelopes = replans.get('applied_envelopes', [])
    result.update(swing_samples=swing_count, geometry_samples=geometry, contract_samples=contract,
                  geometry_fraction=geometry/swing_count if swing_count else None,
                  contract_fraction=contract/swing_count if swing_count else None,
                  changed_from_baseline_samples=corrected, status_counts=dict(statuses),
                  baseline_axis_change_mps2=quantiles(deviations), foot_qp_change_norm_mps2=quantiles(corrections),
                  all_swing_baseline_axis_change_mps2=quantiles(all_swing_deviations),
                  final_constraint_violation_mps2=quantiles(violations), active_empirical_residual_mps2=quantiles(residuals),
                  actual_activation_delay_ms=quantiles(delay), actual_activation_position_mm=quantiles(pos),
                  actual_activation_velocity_mps=quantiles(vel),
                  policy_update_interval_ms=quantiles(np.diff(policy_times)*1000),
                  applied_derivative_bounds={name: quantiles([r[name] for r in envelopes])
                                             for name in ('velocity_mps', 'acceleration_mps2', 'jerk_mps3')},
                  applied_domain_end_error_s=quantiles([r['end_error_s'] for r in envelopes]),
                  diagnostic_gap_ms=quantiles(np.diff([float(r['sim_time']) for r in rows])*1000),
                  wbc_compute_ms=quantiles([float(r['wbc_compute_ms']) for r in rows]),
                  invalid_wbc_samples=sum(float(r['valid']) == 0 for r in rows),
                  final_xy=outcome['final'][1:3], max_abs_roll=outcome['max_abs_roll'],max_abs_pitch=outcome['max_abs_pitch'])
    result['map_evidence'] = map_evidence(path)
    result['obstacle_crossings'] = obstacle_crossings(path,outcome['scenario'])
    crossings = result['obstacle_crossings']
    result['passed_with_all_feet_in_obstacle_span'] = (
        result['result'] == 'passed' and crossings['all_feet_crossed_within_span']) if crossings else None
    final_time = float(rows[-1]['sim_time']) if rows else 0.
    result['last_second_swing_status_counts'] = dict(Counter(
        int(float(row[leg+'_safety_status'])) for row in rows if float(row['sim_time']) >= final_time-1
        for leg, bit in zip(LEGS, (8, 4, 2, 1)) if not int(float(row['mode'])) & bit))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = [analyze(p) for p in args.trials]
    groups = defaultdict(list)
    for result in results:
        key = '|'.join(str(result[k]) for k in ('scenario','mode','online_replan','binary_hash','protocol_hash'))
        # Never merge tuning configurations, worlds or pulse protocols.
        groups[key].append(result)
    group_summary = {}
    for key, trials in groups.items():
        n = len(trials); passed = sum(t['result'] == 'passed' for t in trials)
        group_summary[key] = dict(independent_runs=n, passed=passed, wilson95=wilson(passed,n),
                                  outcomes=dict(Counter(t['result'] for t in trials)),
                                  trials=[t['trial'] for t in trials])
    args.output.write_text(json.dumps(dict(trials=results,groups=group_summary,
        experimental_unit='independent simulator launch; samples are conditional diagnostics, not replicates'),indent=2)+'\n')
    fields=['trial','scenario','mode','online_replan','result','replan_applied','received_wbc_samples','swing_samples',
            'geometry_samples','contract_samples','geometry_fraction','contract_fraction','changed_from_baseline_samples']
    with args.output.with_suffix('.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    for key, group in group_summary.items():
        print(key.split('|')[:3], group['passed'], '/', group['independent_runs'], group['wilson95'])


if __name__ == '__main__':
    main()
