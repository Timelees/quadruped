"""Shared measurement helpers for the independent paper experiment packages."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

LEGS = ('LF', 'RF', 'LH', 'RH')


def workspace():
    for path in (Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents):
        if (path/'src/quadruped_ros2_control').is_dir():
            return path
    raise RuntimeError('Run from the quadruped workspace or pass --workspace')


def arguments(package, parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, default=workspace())
    parser.add_argument('--seeds', type=int, nargs='+', default=[7101, 7102, 7103])
    parser.add_argument('--domain', type=int, default=130)
    parser.add_argument('--analyze-only', action='store_true')
    args = parser.parse_args()
    args.root = args.workspace/'src/quadruped_ros2_control/test'/package
    return args


def trial(args, name, options):
    output = args.root/'data'/f'{name}.json'
    if not args.analyze_only and not output.exists():
        script = args.workspace/'src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/run_trial.py'
        result = subprocess.run([sys.executable, str(script), '--domain', str(args.domain),
                                 '--output', str(output), *map(str, options)], cwd=args.workspace)
        if not output.exists():
            raise RuntimeError(f'Trial infrastructure failed ({result.returncode}): {name}')
    return output


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def numeric(rows, key):
    return np.array([float(row[key]) for row in rows])


def scan_summary(rows):
    """Pair every update with its unchanged trajectory; NaN is unobserved."""
    frozen = {row['case_id']: float(row['clearance_m']) for row in rows if row['method']=='Frozen'}
    summary = {}
    for method in ('Frozen', 'Variable', 'Direct', 'Admitted'):
        selected = [row for row in rows if row['method']==method]
        accepted = [row for row in selected if int(row['admitted'])]
        finite = [row for row in accepted if np.isfinite(float(row['clearance_m']))]
        safe = [row for row in finite if float(row['clearance_m'])>=-1e-6]
        colliding = [row for row in selected if frozen[row['case_id']] < -1e-6]
        outcomes = {}
        for row in selected:
            key = row['outcome']
            outcomes[key] = outcomes.get(key, 0)+1
        summary[method] = dict(conditions=len(selected), accepted=len(accepted),
            finite_clearance_accepted=len(finite), unobserved_clearance_accepted=len(accepted)-len(finite),
            safe_sampled=len(safe), frozen_colliding_conditions=len(colliding),
            rescued_frozen_collisions=sum(frozen[row['case_id']] < -1e-6 for row in safe),
            outcomes=outcomes)
        for key in ('position_jump_m', 'velocity_jump_mps', 'acceleration_jump_mps2', 'deadline_shift_s',
                    'terminal_position_error_m', 'terminal_velocity_error_mps', 'terminal_acceleration_error_mps2'):
            summary[method]['max_'+key] = max((float(row[key]) for row in accepted if key in row),default=None)
    return summary


def contact_events(rows):
    """One event per scheduled swing; keep undetected contacts as missing.

    A physical touchdown needs two consecutive >=5 N samples after a measured
    <1 N airborne sample and 50% of the nominal 0.4 s swing. No force onset is
    inferred across a diagnostic gap >20 ms. Planned deadlines have 5 ms
    recording resolution; this is measured timing, not an exact contact proof.
    """
    t = numeric(rows, 'sim_time')
    mode = numeric(rows, 'mode').astype(int)
    events = []
    for leg, bit in zip(LEGS, (8, 4, 2, 1)):
        stance = (mode & bit) != 0
        force = numeric(rows, 'sensor_'+leg)
        starts = np.flatnonzero(stance[:-1] & ~stance[1:])+1
        for start in starts:
            endings = np.flatnonzero(stance[start:])
            if not len(endings):
                continue
            end = start+endings[0]
            search = np.flatnonzero((t >= t[start]) & (t <= t[end]+.15))
            airborne = np.flatnonzero((force[start:end] < 1.0))
            contact = None
            if len(airborne):
                for index in search:
                    if index+1 < len(t) and t[index] >= t[start]+.2 and t[index] > t[start+airborne[0]] and \
                            np.all(np.diff(t[start:index+2]) <= .020) and \
                            min(force[index:index+2]) >= 5. and force[index-1] < 5.:
                        contact = float(t[index])
                        break
            event = dict(leg=leg, liftoff_s=float(t[start]), deadline_s=float(t[end]),
                         contact_s=contact, timing_error_s=None if contact is None else contact-t[end],
                         phase_error_swing_fraction=None if contact is None else (contact-t[end])/(t[end]-t[start]))
            if contact is not None and leg+'_p_z' in rows[index]:
                event['estimated_contact_vertical_error_m'] = float(rows[index][leg+'_p_z'])-float(rows[end][leg+'_pd_z'])
                event['estimated_contact_vertical_velocity_mps'] = float(rows[index][leg+'_v_z'])
            events.append(event)
    return events


def measure(path):
    result = json.loads(path.read_text())
    manifest = json.loads(path.with_suffix('.manifest.json').read_text())
    wbc = read_csv(path.with_name(path.stem+'_wbc.csv'))
    body = read_csv(path.with_suffix('.csv'))
    moving = [row for row in body if row['phase'] in ('forward', 'hold')]
    out = dict(trial=path.stem, result=result['result'], scenario=result['scenario'],
               seed=manifest['trial.seed'], online_replan=result['online_replan'],
               safety_filter=result['safety_filter'], replan_applied=result['replan_applied'],
               controller_sha256=manifest.get('install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so'))
    out['max_roll_rad'] = float(np.max(np.abs(numeric(moving, 'roll')))) if moving else None
    out['max_pitch_rad'] = float(np.max(np.abs(numeric(moving, 'pitch')))) if moving else None
    out['invalid_wbc_samples'] = int(np.sum(numeric(wbc, 'valid') == 0))
    stamp = numeric(wbc, 'sim_time')
    out['diagnostic_gaps_over_20ms'] = int(np.sum(np.diff(stamp) > .020))
    events = contact_events(wbc)
    confirm_contact_geometry(path, wbc, events)
    path.with_suffix('.contacts.json').write_text(json.dumps(events, indent=2)+'\n')
    errors = [event['timing_error_s'] for event in events if event['contact_s'] is not None]
    out['scheduled_swings'] = len(events)
    out['detected_touchdowns'] = len(errors)
    out['touchdown_abs_error_median_ms'] = float(np.median(np.abs(errors))*1000) if errors else None
    out['touchdown_abs_error_p95_ms'] = float(np.percentile(np.abs(errors), 95)*1000) if errors else None
    out['touchdown_signed_error_mean_ms'] = float(np.mean(errors)*1000) if errors else None
    phase_errors = [event['phase_error_swing_fraction'] for event in events if event['contact_s'] is not None]
    out['touchdown_abs_swing_phase_error_median'] = float(np.median(np.abs(phase_errors))) if phase_errors else None
    physical_errors = [event['timing_error_s'] for event in events if event.get('target_contact_confirmed')]
    out['geometry_confirmed_target_contacts'] = len(physical_errors)
    out['physical_contact_abs_error_median_ms'] = float(np.median(np.abs(physical_errors))*1000) if physical_errors else None
    out['physical_contact_abs_error_p95_ms'] = float(np.percentile(np.abs(physical_errors),95)*1000) if physical_errors else None
    # Exclude initialization/stand using the ROS clock recorded with each tick.
    start = float(moving[0]['sim_time']) if moving else float('inf')
    selected = [row for row in wbc if float(row['ros_time']) >= start]
    if selected and 'measured_com_x' in selected[0]:
        error = np.array([[float(row[f'measured_com_{a}'])-float(row[f'desired_com_{a}'])
                           for a in 'xyz'] for row in selected])
        out['model_com_tracking_rms_m'] = float(np.sqrt(np.mean(np.sum(error*error, axis=1))))
        out['model_com_tracking_xy_rms_m'] = float(np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1))))
    if len(selected) > 1:
        forces = np.array([[float(row[f'force_{i}']) for i in (2, 5, 8, 11)] for row in selected])
        dt = np.diff(numeric(selected, 'sim_time'))
        contiguous = (dt >= .0025) & (dt <= .0075)
        out['support_force_step_pairs'] = int(contiguous.sum())
        out['support_force_step_rms_N'] = float(np.sqrt(np.mean(np.sum(np.diff(forces, axis=0)[contiguous]**2, axis=1)))) if contiguous.any() else None
    forward = [row for row in body if row['phase'] == 'forward']
    if forward:
        duration = float(forward[-1]['sim_time'])-float(forward[0]['sim_time'])
        displacement = np.linalg.norm([float(forward[-1][a])-float(forward[0][a]) for a in 'xy'])
        out['route_average_speed_mps'] = float(displacement/duration) if duration > 0 else None
    if result['scenario'] in ('obstacle', 'course'):
        from analyze_hocbf_trials import obstacle_crossings
        crossing = obstacle_crossings(path, result['scenario'])
        path.with_suffix('.crossings.json').write_text(json.dumps(crossing, indent=2)+'\n')
        out['all_feet_crossed_within_span'] = crossing['all_feet_crossed_within_span']
        observed = [r['top_clearance_m'] for r in crossing['crossings'] if r.get('within_span') and r['pose_gap_s'] <= .020]
        out['minimum_crossing_top_clearance_m'] = min(observed) if observed else None
    return out


def confirm_contact_geometry(path, rows, events):
    """Cross-check force onsets against native geometry and the terminal site.

    The joint force-torque norm also includes inertia and obstacle impacts. It
    alone cannot establish touchdown. Keep every unconfirmed event explicit.
    """
    from analyze_true_geometry import true_feet, world_boxes, signed_clearance
    stamps, positions = true_feet(path.with_name(path.stem+'_gz_poses.csv'))
    boxes = world_boxes(path.with_suffix('.world.sdf'))
    controller = numeric(rows,'sim_time')
    for event in events:
        event['target_contact_confirmed'] = False
        if event['contact_s'] is None:
            event['contact_geometry_reason'] = 'force onset unobserved or diagnostic gap'
            continue
        index = int(np.argmin(np.abs(controller-event['contact_s'])))
        end = int(np.argmin(np.abs(controller-event['deadline_s'])))
        ros_time = float(rows[index]['ros_time'])
        frame = int(np.argmin(np.abs(stamps-ros_time)))
        gap = abs(float(stamps[frame])-ros_time)
        leg = event['leg']
        p = positions[frame,LEGS.index(leg)]
        target = np.array([float(rows[end][leg+'_pd_'+axis]) for axis in 'xyz'])
        clearance = float(signed_clearance(p[None,:],boxes)[0])
        error = float(np.linalg.norm(p-target))
        confirmed = gap <= .012 and abs(clearance) <= .003 and error <= .040
        event.update(target_contact_confirmed=bool(confirmed), native_pose_gap_s=gap,
                     true_surface_clearance_m=clearance, true_terminal_distance_m=error,
                     contact_geometry_reason='confirmed' if confirmed else 'pose gap, surface gap or terminal distance exceeds protocol')


def summarize(args, paths):
    rows = [measure(path) for path in paths if path.exists()]
    (args.root/'data/summary.json').write_text(json.dumps(rows, indent=2)+'\n')
    if rows:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        with (args.root/'data/summary.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    return rows


def freeze(args, sources):
    manifest = {}
    for path in sources:
        manifest[str(path.relative_to(args.workspace))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.root/'data/analysis_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


def plot_metrics(root, rows, fields):
    """Show every independent run, retaining failed runs and missing metrics."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 7,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none',
                         'axes.spines.top': False, 'axes.spines.right': False})
    figure, axes = plt.subplots(1, len(fields), figsize=(7.2, 2.8), squeeze=False)
    figure.subplots_adjust(left=.09, right=.98, bottom=.36, top=.90, wspace=.65)
    lateral = all(row['scenario'] == 'lateral' for row in rows)
    groups = ['Left', 'Right'] if lateral else ['Frozen', 'Online', 'Online + filter']
    def group(row):
        if lateral:
            return 'Left' if '_left_' in row['trial'] else 'Right'
        return 'Frozen' if row['online_replan'] == 'off' else ('Online' if row['safety_filter'] == 'off' else 'Online + filter')
    seeds = sorted(set(row['seed'] for row in rows))
    offsets = dict(zip(seeds, np.linspace(-.14, .14, len(seeds))))
    for letter, panel, (key, label, scale) in zip('abcdef', axes[0], fields):
        maximum = 0.
        for i, row in enumerate(rows):
            value = row.get(key)
            if value is not None and np.isfinite(value):
                x = groups.index(group(row)) + offsets[row['seed']]
                panel.scatter(x, value*scale, color='#376b91' if row['result']=='passed' else '#b36b47',
                              marker='o' if row['result']=='passed' else 'x', s=18)
                maximum = max(maximum, value*scale)
        panel.set_xticks(range(len(groups)), [g.replace(' + ', '\n+ ') for g in groups])
        panel.set_xlim(-.5, len(groups)-.5)
        panel.text(-.18, 1.06, letter, transform=panel.transAxes, fontweight='bold')
        panel.set_ylabel(label)
        panel.set_ylim(0, maximum*1.15 if maximum else 1.)
    save_figure(figure, root/'figure/results')


def save_figure(figure, path, **alignment):
    import matplotlib.pyplot as plt
    scripts = Path(os.environ.get('NATURE_FIGURE_SCRIPTS',
                                 Path.home()/'.codex/skills/nature-figure/scripts'))
    sys.path.insert(0, str(scripts))
    from audit_panel_alignment import require_matplotlib_panel_alignment
    figure.canvas.draw()
    require_matplotlib_panel_alignment(figure, json_out=str(path)+'.alignment.json', strict=True, **alignment)
    figure.savefig(str(path)+'.pdf')
    figure.savefig(str(path)+'.svg')
    figure.savefig(str(path)+'.png', dpi=600)
    plt.close(figure)
    for script, options in [('audit_pdf_text.py', ['--min-pt', '5']),
                            ('audit_figure_collisions.py', ['--json-out', str(path)+'.collision-audit.json'])]:
        qa_python = os.environ.get('NATURE_FIGURE_QA_PYTHON', sys.executable)
        subprocess.run([qa_python, str(scripts/script), str(path)+'.pdf', *options], check=True)
