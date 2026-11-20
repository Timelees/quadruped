#!/usr/bin/env python3
"""Acceptance measurements from explicit launch result paths, including failures.

Samples describe observed nominal constraints and can falsify model budgets.
They cannot establish a deterministic physical bound or continuous-time safety.
"""
import argparse
import csv
import json
from pathlib import Path
import re
from collections import Counter

import numpy as np
from analyze_hocbf_trials import quantiles, map_evidence, obstacle_crossings
from analyze_true_geometry import true_feet

LEGS = ('LF','RF','LH','RH')


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def certificate_durations(rows, max_gap=.020):
    """Count only observed adjacent intervals; do not fill diagnostic dropouts."""
    durations = {str(state): 0.0 for state in (0, 1, 2)}
    gaps = []
    intervals = []
    for previous, row in zip(rows, rows[1:]):
        start, end = float(previous['sim_time']), float(row['sim_time'])
        dt = end-start
        if dt <= 0 or dt > max_gap:
            if dt > max_gap: gaps.append(dt)
            continue
        for leg in LEGS:
            state = int(float(previous[leg+'_certificate_state']))
            durations[str(state)] += dt
            if state == 2:
                intervals.append(dict(leg=leg, start_s=start, end_s=end))
    # Merge per-leg adjacent intervals, retaining holes as holes.
    merged = []
    for leg in LEGS:
        for interval in (item for item in intervals if item['leg']==leg):
            if merged and merged[-1]['leg']==leg and abs(merged[-1]['end_s']-interval['start_s'])<1e-8:
                merged[-1]['end_s'] = interval['end_s']
            else:
                merged.append(dict(interval))
    return dict(observed_leg_seconds=durations, uncertified_intervals=merged,
                omitted_gap_seconds=sum(gaps), omitted_gaps=len(gaps), max_included_gap_s=max_gap)


def reveal_window(trigger, rows, applied, direct, activation, launch_text):
    """Keep evidence in the triggered LF swing, never borrow later updates."""
    start = trigger['controller_time']
    # Historical protocol used .4 s; new trials record the isolated gait duration.
    end = trigger['liftoff'] + trigger.get('swing_duration_s', .4)
    selected = [row for row in rows if trigger['liftoff'] <= float(row['sim_time']) < end]
    foot_x = [float(row['LF_p_x']) for row in selected]
    attempts = []
    # RCLCPP and std::cerr share stderr. Remove only complete RCLCPP records
    # and launch line prefixes; preserve the planner's actual fields even when
    # an unrelated log line interrupts them. Never infer a missing decision.
    planner_text = re.sub(r'(?m)^\[[^\n]+?\] ', '', launch_text)
    planner_text = re.sub(r'\[(?:INFO|WARN|ERROR|DEBUG|FATAL)\] \[[^\n]+?\] \[[^\n]+?\]:[^\n]*\n', '', planner_text)
    pattern = r'Swing replan leg=\s*(\d+)\s+activation=(\S+).*?outcome=(\d+) reason=(.*?) replan_ms=(\S+)'
    records = list(re.finditer(pattern, planner_text))
    for match in records:
        time = float(match[2])
        if int(match[1]) == 0 and start <= time < end:
            attempts.append(dict(time_s=time, outcome=int(match[3]), reason=match[4],
                                 computation_ms=float(match[5])))
    return dict(start_s=start, end_s=end, liftoff_s=trigger['liftoff'],
        observed_LF_x_range_m=[min(foot_x), max(foot_x)] if foot_x else None,
        applied=[r for r in applied if r['leg']==0 and start <= r['applied'] < end],
        direct=[r for r in direct if r['leg']==0 and start <= r['time'] < end],
        activation_reasons=dict(Counter(r['reason'] for r in activation
            if r['leg']==0 and start <= r['actual_time'] < end)),
        planner_attempts=attempts,
        unparsed_planner_records_total=planner_text.count('Swing replan leg=')-len(records))


def masked_unknown_samples(rows, events, rectangle):
    """Audit feet strictly inside a persistently masked unknown strip.

    Discard the first .3 s for message propagation. Spatially erode by .04 m
    so grid edges cannot be confused with a query entirely inside the mask.
    """
    masks = [event for event in events if event['event']=='mask_applied']
    releases = [event for event in events if event['event']=='map_released']
    if not masks:
        return dict(available=False, reason='No observed mask application')
    start = masks[0]['ros_time']+.3
    end = releases[0]['received_ros_time'] if releases else float('inf')
    xmin,xmax,ymin,ymax = rectangle
    counts = Counter()
    geometry_active = 0
    for row in rows:
        if not start <= float(row['ros_time']) < end:
            continue
        mode = int(float(row['mode']))
        for leg,bit in zip(LEGS,(8,4,2,1)):
            if mode & bit:
                continue
            x,y = float(row[leg+'_p_x']),float(row[leg+'_p_y'])
            if xmin+.04 < x < xmax-.04 and ymin+.04 < y < ymax-.04:
                counts[str(int(float(row[leg+'_certificate_state'])))] += 1
                geometry_active += int(int(float(row[leg+'_safety_status'])) in (1,13,15,16))
    return dict(available=True, rectangle_m=list(rectangle), spatial_erosion_m=.04,
        propagation_exclusion_s=.3, observed_swing_leg_samples=sum(counts.values()),
        certificate_states=dict(counts), certified_samples=counts['1'],
        geometry_active_samples=geometry_active, revealed=bool(releases),
        definition='Finite point audit inside test-only unknown strip; not a whole-curve certificate')


def analyze(path):
    result = json.loads(path.read_text())
    manifest = json.loads(path.with_suffix('.manifest.json').read_text())
    out = dict(trial=path.stem, result=result['result'], scenario=result['scenario'],
               safety_filter=result.get('safety_filter'), seed=manifest.get('trial.seed'),
               transport_delay_s=result.get('mrt_transport_delay_s'),
               method=result.get('swing_update_method'), replan_applied=result.get('replan_applied'),
               binary=manifest.get('install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so'))
    rows = read_csv(path.with_suffix('.csv'))
    out['stability'] = {}
    for phase in ('forward','hold','trot','hold_first_5s'):
        selected = [r for r in rows if r['phase']==('hold' if phase=='hold_first_5s' else phase)]
        if not selected: continue
        if phase=='hold_first_5s':
            start = float(selected[0]['sim_time'])
            selected = [r for r in selected if float(r['sim_time'])-start <= 5.0]
        angles = np.array([[float(r[k]) for k in ('roll','pitch')] for r in selected])
        rates = np.array([[float(r[k]) for k in ('wx','wy','wz')] for r in selected])
        xy = np.array([[float(r[k]) for k in ('x','y')] for r in selected])
        out['stability'][phase] = dict(samples=len(selected), duration_s=float(selected[-1]['sim_time'])-float(selected[0]['sim_time']),
            attitude_rms_deg=np.rad2deg(np.sqrt(np.mean(angles**2,axis=0))).tolist(),
            attitude_max_deg=np.rad2deg(np.max(np.abs(angles),axis=0)).tolist(),
            angular_rate_rms_rps=np.sqrt(np.mean(rates**2,axis=0)).tolist(),
            drift_from_first_m=float(np.linalg.norm(xy[-1]-xy[0])),
            max_displacement_from_first_m=float(np.max(np.linalg.norm(xy-xy[0],axis=1))))
    wbc = read_csv(path.with_name(path.stem+'_wbc.csv'))
    if wbc:
        out['invalid_qp_samples'] = sum(float(r['valid'])!=1 for r in wbc)
        out['control_period_s'] = quantiles([float(r['control_period']) for r in wbc])
        out['wbc_compute_ms'] = quantiles([float(r['wbc_compute_ms']) for r in wbc])
        if 'mrt_residence_s' in wbc[0]:
            out['actual_transport_residence_s'] = quantiles([float(r['mrt_residence_s']) for r in wbc])
            out['transport_dropped_packages'] = max(float(r['mrt_dropped']) for r in wbc)
            out['actuator_model_valid_fraction'] = float(np.mean([float(r['actuator_model_valid']) for r in wbc]))
        active = [r for r in wbc if any(int(float(r[leg+'_safety_status'])) in (1,13,15,16) for leg in LEGS)]
        out['nominal_safety_active_wbc_samples'] = len(active)
        out['nominal_safety_row_violation'] = quantiles([-float(r[leg+'_wbc_safety_residual']) for r in active
            for leg in LEGS if int(float(r[leg+'_safety_status'])) in (1,13,15,16)])
        if 'LF_residual_bound_lower' in wbc[0]:
            task = path.with_suffix('.task.info').read_text()
            disturbance = float(re.search(r'^\s*safetyFilterDisturbanceBound\s+(\S+)',task,re.M)[1])
            lower = np.array([float(r[leg+'_residual_bound_lower']) for r in wbc for leg in LEGS])
            lower = lower[np.isfinite(lower)]
            out['average_disturbance_lower_bound_mps2'] = quantiles(lower)
            out['disturbance_budget_mps2'] = disturbance
            out['residual_budget_interpretation'] = (
                'Conditional average mismatch lower bound assuming the configured velocity measurement '
                'error and hold model. Exceedances contradict the joint model/measurement/hold budget; '
                'they do not identify acceleration disturbance alone or establish a deterministic bound.')
            out['observed_budget_falsification_samples'] = int(np.sum(lower>disturbance+1e-6))
            interior=[]
            for previous,row in zip(wbc,wbc[1:]):
                for leg in LEGS:
                    if all(int(float(r[leg+'_certificate_state']))==1 and float(r['sensor_'+leg])<1.0 for r in (previous,row)):
                        value=float(row[leg+'_residual_bound_lower'])
                        if np.isfinite(value): interior.append(value)
            out['observed_contact_free_interior_disturbance_lower_bound_mps2'] = quantiles(interior)
            out['interior_budget_falsification_samples'] = sum(v>disturbance+1e-6 for v in interior)
            out['certified_noncontact_leg_samples'] = sum(int(float(r[leg+'_certificate_state']))==1 for r in wbc for leg in LEGS)
            out['uncertified_noncontact_leg_samples'] = sum(int(float(r[leg+'_certificate_state']))==2 for r in wbc for leg in LEGS)
            out['certificate_durations'] = certificate_durations(wbc)
    if wbc and 'actuator_model_valid' in wbc[0]:
        torque_mismatch=[]
        for previous,row in zip(wbc,wbc[1:]):
            if float(row['ros_time'])-float(previous['ros_time'])>.006: continue
            if any(int(float(previous[leg+'_safety_status'])) in (1,13,15,16) for leg in LEGS):
                torque_mismatch.append(max(abs(float(row[f'applied_torque_{i}'])-float(previous[f'predicted_torque_{i}'])) for i in range(12)))
        out['next_sample_torque_mismatch_Nm'] = quantiles(torque_mismatch)
        stamps,positions=true_feet(path.with_name(path.stem+'_gz_poses.csv'))
        velocities=np.gradient(positions,stamps,axis=0)
        ros=np.array([float(r['ros_time']) for r in wbc])
        index=np.searchsorted(stamps,ros)
        matched=(index>0)&(index<len(stamps))
        index=np.clip(index,1,len(stamps)-1)
        matched&=(stamps[index]-stamps[index-1] <= .020)
        position_error=[];velocity_error=[]
        for leg_index,leg in enumerate(LEGS):
            mask=matched&np.array([int(float(row[leg+'_certificate_state']))==1 and float(row['sensor_'+leg])<1.0 for row in wbc])
            true_p=np.column_stack([np.interp(ros,stamps,positions[:,leg_index,axis]) for axis in range(3)])
            true_v=np.column_stack([np.interp(ros,stamps,velocities[:,leg_index,axis]) for axis in range(3)])
            measured_p=np.array([[float(row[f'{leg}_p_{axis}']) for axis in 'xyz'] for row in wbc])
            measured_v=np.array([[float(row[f'{leg}_v_{axis}']) for axis in 'xyz'] for row in wbc])
            position_error.extend(np.linalg.norm(true_p-measured_p,axis=1)[mask])
            velocity_error.extend(np.linalg.norm(true_v-measured_v,axis=1)[mask])
        out['sampled_state_measurement_check'] = dict(position_error_m=quantiles(position_error),
            approximate_velocity_error_mps=quantiles(velocity_error),
            definition='World-composed Gazebo foot poses, linear interpolation across <=20ms; velocity by native-pose central difference; certified noncontact and sensor force <1N only; finite approximate audit, not deterministic calibration')
    out['activation'] = json.loads(path.with_suffix('.activation.json').read_text()) if path.with_suffix('.activation.json').exists() else []
    text = path.with_suffix('.launch.log').read_text(errors='replace')
    out['transport_releases'] = [dict(time=float(m[0]),source=float(m[1]),residence_s=float(m[2])) for m in
        re.findall(r'MRT transport release now=(\S+) source=(\S+) residence=(\S+)',text)]
    task = path.with_suffix('.task.info').read_text()
    start_time = float(re.search(r'^\s*mrtTransportStartTime\s+(\S+)',task,re.M)[1])
    out['injected_transport_residence_s'] = quantiles([r['residence_s'] for r in out['transport_releases'] if r['time']>=start_time])
    out['activation_reasons'] = dict(Counter(r['reason'] for r in out['activation']))
    out['joint_admission_rejections'] = text.count('reason=activation_joint_wbc')
    out['direct_activations'] = [dict(leg=int(m[0]),time=float(m[1]),position_error_m=float(m[2]),velocity_error_mps=float(m[3]),reference_jump_m=float(m[4]))
        for m in re.findall(r'Swing direct activation leg=(\d+) now=(\S+) p_error=(\S+) v_error=(\S+) reference_jump=(\S+)',text)]
    out['reveal'] = json.loads(path.with_suffix('.reveal.json').read_text()) if path.with_suffix('.reveal.json').exists() else []
    if manifest.get('trial.reveal',{}).get('unknown'):
        centre = manifest['trial.controlled_bars'][1][0]
        out['unknown_strip_audit'] = masked_unknown_samples(wbc,out['reveal'],(centre-.2,centre+.2,-3.7,-2.3))
    replans = json.loads(path.with_suffix('.replans.json').read_text())
    out['applied_references'] = replans['applied']
    triggers = [r for r in out['reveal'] if r['event']=='trigger']
    if triggers:
        trigger = triggers[0]
        window = reveal_window(trigger, wbc, replans['applied'], out['direct_activations'], out['activation'], text)
        # Independently verify actual crossing from native Gazebo poses.
        stamps, positions = true_feet(path.with_name(path.stem+'_gz_poses.csv'))
        offset = trigger['ros_time']-trigger['controller_time']
        mask = (stamps >= trigger['liftoff']+offset) & (stamps < window['end_s']+offset)
        x = positions[mask, 0, 0]
        window['true_LF_x_range_m'] = [float(x.min()),float(x.max())] if len(x) else None
        front = trigger['barrier_x']-.5*trigger['barrier_width']-.019
        back = trigger['barrier_x']+.5*trigger['barrier_width']+.019
        window['true_LF_spanned_bar'] = bool(len(x) and x.min()<front and x.max()>back)
        window['pose_samples'] = int(mask.sum())
        releases = [r for r in out['reveal'] if r['event']=='map_released']
        window['released_map_age_s'] = releases[0]['received_ros_time']-releases[0]['map_stamp'] if releases else None
        out['reveal_swing_window'] = window
    out['map_evidence'] = map_evidence(path)
    from analyze_map_envelope import analyze as map_envelope
    out['map_prism_envelope'] = map_envelope(path)
    out['crossings'] = obstacle_crossings(path,result['scenario'])
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    results=[analyze(path) for path in args.trials]
    args.output.write_text(json.dumps(dict(trials=results, caveat='Finite observations; retain failures. Nominal rows and sampled disturbance checks are distinct.'),indent=2,allow_nan=False)+'\n')


if __name__=='__main__': main()
