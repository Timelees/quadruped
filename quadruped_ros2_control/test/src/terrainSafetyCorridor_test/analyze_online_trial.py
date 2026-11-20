#!/usr/bin/env python3
"""Summarize executed swing constraints and draw evidence from Gazebo trial logs."""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

LEGS = ('LF', 'RF', 'LH', 'RH')
STATUS = ('Off', 'HOCBF', 'No corridor', 'Lift-off / outside', 'Foot QP infeasible',
          'Invalid state', 'Period limit', 'Barrier entry', 'Tube entry', 'Tube rate',
          'Tube margin', 'WBC fallback', 'Stance', 'Landing', 'No landing corridor',
          'Geometry only', 'Landing geometry only')
AUDIT_TOOLS = None


def save(figure, stem):
    if AUDIT_TOOLS is not None:
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(
            figure, json_out=stem.with_suffix('.alignment.json'), strict=True)
    for extension in ('.pdf', '.png', '.svg'):
        figure.savefig(stem.with_suffix(extension), dpi=300)
    plt.close(figure)


def finite(values):
    values = np.asarray(values, dtype=float)
    return values[np.isfinite(values)]


def quantiles(values):
    return np.percentile(values, [50, 95, 99]).tolist() if len(values) else None


def touchdown_events(rows):
    """Detect force onset around the nearest scheduled swing-to-stance switch."""
    if not rows or 'sensor_LF' not in rows[0]:
        return []
    time = np.array([float(r['sim_time']) for r in rows])
    modes = np.array([int(float(r['mode'])) for r in rows])
    events = []
    for leg, bit in zip(LEGS, (8, 4, 2, 1)):
        force = np.array([float(r[f'sensor_{leg}']) for r in rows])
        velocity = np.array([float(r[f'{leg}_v_z']) for r in rows])
        stance = (modes & bit) != 0
        for i in np.flatnonzero(~stance[:-1] & stance[1:]) + 1:
            if time[i] - time[i-1] > .025:
                continue
            scheduled = (time[i] + time[i-1]) / 2
            rising = np.flatnonzero((force[:-1] <= 3.) & (force[1:] > 3.)) + 1
            hits = rising[(time[rising] >= scheduled-.10) & (time[rising] <= scheduled+.08)]
            hits = hits[(time[hits] - time[hits-1]) <= .025]
            if not len(hits):
                continue
            j = hits[np.argmin(np.abs(time[hits] - scheduled))]
            events.append(dict(leg=leg, scheduled_s=scheduled, observed_s=time[j],
                               timing_ms=1000*(time[j]-scheduled), precontact_vz_mps=velocity[j-1]))
    return sorted(events, key=lambda e: e['scheduled_s'])


def plot_touchdown(rows, events, stem):
    # Select the event closest to the median signed timing error.
    median = np.median([e['timing_ms'] for e in events])
    event = min(events, key=lambda e: abs(e['timing_ms'] - median))
    leg = event['leg']
    scheduled = event['scheduled_s']
    window = [r for r in rows if abs(float(r['sim_time']) - scheduled) <= .12]
    time = [1000 * (float(r['sim_time']) - scheduled) for r in window]
    fig, (force, velocity) = plt.subplots(2, 1, figsize=(3.7, 3.2), sharex=True, layout='constrained')
    force.plot(time, [float(r[f'sensor_{leg}']) for r in window], '.-', color='#157285', ms=3)
    force.axhline(3., color='#a45b23', ls=':', label='Force threshold')
    force.set(ylabel='Contact force (N)')
    velocity.plot(time, [float(r[f'{leg}_v_z']) for r in window], '.-', color='#157285', ms=3)
    velocity.set(xlabel='Time from scheduled switch (ms)', ylabel='Vertical speed (m/s)')
    landing = [float(r.get(f'{leg}_safety_status', -1)) in (13, 16) for r in window]
    for ax in (force, velocity):
        ax.axvline(0, color='0.35', lw=.8, label='Scheduled switch')
        ax.axvline(event['timing_ms'], color='#a45b23', ls='--', lw=.8, label='Detected contact')
        ax.fill_between(time, 0, 1, where=landing, step='mid', transform=ax.get_xaxis_transform(),
                        color='#157285', alpha=.13, label='Landing constraints')
    force.legend(fontsize=7, loc='lower left', bbox_to_anchor=(0, 1.02), ncol=2,
                 title=f'{leg} foot near scheduled touchdown', title_fontsize=8)
    save(fig, stem)


def plot_sweeps(directory):
    with (directory / 'admission_sweep.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    delays = sorted({float(r['delay_ms']) for r in rows})
    fig, axes = plt.subplots(1, len(delays), figsize=(7.1, 2.6), sharey=True, layout='constrained')
    scatter = None
    for ax, delay in zip(axes, delays):
        groups = {}
        for row in rows:
            if float(row['delay_ms']) == delay:
                key = (400 * (1 - float(row['phase'])) - delay, float(row['position_shift_mm']))
                groups.setdefault(key, []).append(int(row['outcome']) == 1)
        scatter = ax.scatter(*zip(*groups), c=[np.mean(g) for g in groups.values()], s=75,
                             marker='s', cmap='viridis', vmin=0, vmax=1)
        ax.set(title=f'{delay:g} ms delay', xlabel='Remaining swing (ms)')
        ax.set_xticks(sorted({k[0] for k in groups}))
    axes[0].set_ylabel('Vertical position shift (mm)')
    fig.colorbar(scatter, ax=axes, label='Admission fraction', ticks=[0, .5, 1])
    save(fig, directory / 'admission_sweep')

    with (directory / 'disturbance_sweep.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    fig, (clearance, tube) = plt.subplots(1, 2, figsize=(7.1, 2.6), layout='constrained')
    for mode, label, color, marker in [('0', 'Tracking', '#737c85', 'x'), ('1', 'HOCBF', '#a45b23', 's'),
                                       ('2', 'HOCBF + tube', '#157285', 'o')]:
        points = [r for r in rows if r['mode'] == mode]
        pulses = [float(r['pulse_mps2']) for r in points]
        clearance.plot(pulses, [float(r['minimum_clearance_mm']) for r in points],
                       marker=marker, ms=5, fillstyle='none', color=color, label=label)
        if mode == '2':
            tube.plot(pulses, [int(r['tube_samples']) / 80 for r in points], marker=marker, color=color)
    clearance.set(xlabel='Downward pulse (m/s²)', ylabel='Minimum clearance (mm)')
    clearance.legend()
    tube.set(xlabel='Downward pulse (m/s²)', ylabel='Fraction with tube enforced', ylim=(0, 1.05))
    save(fig, directory / 'disturbance_sweep')


def compare_trials(directory, prefix):
    records = []
    for path in sorted(directory.glob(f'{prefix}*.metrics.json')):
        row = json.loads(path.read_text())
        trial = json.loads(path.with_name(path.name.replace('.metrics', '')).read_text())
        counts = row.get('safety_counts', {})
        records.append(dict(trial=path.stem.replace('.metrics', ''), scenario=row['scenario'],
                            result=row['result'], safety=trial['safety_filter'], online=trial['online_replan'],
                            wbc_samples=row['wbc_samples'],
                            decisions=sum(row['replan_outcomes'].values()),
                            admitted=row['replan_outcomes'].get('1', 0), applied=row['replans_applied'],
                            swing_samples=sum(counts.values()), constrained=row['filtered'],
                            tube=row['tube_enforced'], corrected=row['corrected'],
                            invalid_wbc=row['invalid_wbc_samples'],
                            wbc_p99_ms=(row.get('wbc_compute_ms_p50_p95_p99') or [None]*3)[2],
                            wbc_exceeded_samples=row.get('wbc_compute_exceeds_period_samples'),
                            wbc_residual=row['worst_sampled_wbc_safety_residual']))
    if not records:
        raise ValueError(f'No {prefix}*.metrics.json files to compare')
    with (directory / 'comparison.csv').open('w') as handle:
        writer = csv.DictWriter(handle, list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    trot = [r for r in records if r['scenario'] == 'trot']
    if trot:
        fig, ax = plt.subplots(figsize=(3.6, 2.7), layout='constrained')
        for row in trot:
            with (directory / (row['trial'] + '_wbc.csv')).open() as handle:
                samples = list(csv.DictReader(handle))
            if not samples or 'wbc_compute_ms' not in samples[0]:
                continue
            times = np.sort([float(r['wbc_compute_ms']) for r in samples])
            if not np.all(np.isfinite(times)) or not np.all(times > 0):
                raise ValueError('Runtime CDF needs positive, finite measured times')
            label, color = {'off': ('Tracking', '#737c85'), 'hocbf': ('HOCBF', '#a45b23'),
                            'full': ('HOCBF + tube', '#157285')}[row['safety']]
            ax.step(times, np.arange(1, len(times)+1) / len(times), where='post', color=color,
                    label=f'{label} (n={len(times)})')
        ax.axvline(5, color='0.4', ls='--', lw=.8, label='Control period')
        ax.set(xscale='log', xlabel='Measured WBC time (ms)', ylabel='Empirical cumulative fraction', ylim=(0, 1.02))
        ax.legend(fontsize=7, loc='lower left', bbox_to_anchor=(0, 1.02), ncol=2)
        save(fig, directory / 'runtime')
    # A sample is a published leg/time pair, not an independent repetition.
    records = [r for r in records if r['safety'] == 'full' and r['online'] == 'on']
    fig, (admission, coverage) = plt.subplots(1, 2, figsize=(7.2, 3), layout='constrained')
    y = np.arange(len(records))
    labels = [r['scenario'].capitalize() + (' repeat' if r['trial'].endswith('repeat') else '') +
              (' (failed)' if r['result'] != 'passed' else '') for r in records]
    admission.barh(y, [100 * r['admitted'] / max(1, r['decisions']) for r in records], color='#157285')
    admission.set(yticks=y, yticklabels=labels, xlabel='Admission / logged decisions (%)', xlim=(0, 100))
    for offset, key, label, color in [(-.18, 'constrained', 'Constraints enforced', '#157285'),
                                     (.18, 'corrected', 'Nonzero correction', '#a45b23')]:
        coverage.barh(y + offset, [100 * r[key] / max(1, r['swing_samples']) for r in records],
                      height=.34, label=label, color=color)
    coverage.set(yticks=y, yticklabels=labels, xlabel='Published swing-leg samples (%)', xlim=(0, 100))
    coverage.legend(loc='lower center', bbox_to_anchor=(.5, 1.02), fontsize=7)
    save(fig, directory / 'comparison')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('trial', type=Path, nargs='?')
    inputs.add_argument('--sweeps', type=Path, help='Directory containing the two self-check sweep CSVs')
    inputs.add_argument('--compare', type=Path, help='Directory containing trial metrics')
    parser.add_argument('--prefix', default='final', help='Trial filename prefix for --compare')
    parser.add_argument('--audit-tools', type=Path, help='Directory containing audit_panel_alignment.py')
    args = parser.parse_args()
    global AUDIT_TOOLS
    AUDIT_TOOLS = args.audit_tools
    if AUDIT_TOOLS is not None:
        sys.path.insert(0, str(AUDIT_TOOLS))
    plt.rcParams.update({'font.size': 8, 'pdf.fonttype': 42, 'axes.spines.right': False,
                         'axes.spines.top': False, 'legend.frameon': False, 'svg.fonttype': 'none',
                         'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans']})
    if args.sweeps:
        plot_sweeps(args.sweeps)
        return
    if args.compare:
        compare_trials(args.compare, args.prefix)
        return
    trial = args.trial
    meta = json.loads(trial.read_text())
    replan_path = trial.with_suffix('.replans.json')
    replans = json.loads(replan_path.read_text()) if replan_path.exists() else {'applied': [], 'decisions': []}
    source = trial.with_name(trial.stem + '_wbc.csv')
    with source.open() as handle:
        rows = list(csv.DictReader(handle))
    result = {'result': meta['result'], 'scenario': meta['scenario'], 'wbc_samples': len(rows),
              'replan_outcomes': dict(Counter(record['outcome'] for record in replans['decisions'])),
              'replan_compute_ms_p50_p95_p99': quantiles([r['compute_ms'] for r in replans['decisions']]),
              'replans_applied': len(replans['applied'])}
    events = touchdown_events(rows)
    if rows and 'wbc_compute_ms' in rows[0]:
        timing = [float(r['wbc_compute_ms']) for r in rows]
        result['wbc_compute_ms_p50_p95_p99'] = quantiles(timing)
        result['wbc_compute_exceeds_period_samples'] = sum(
            float(r['wbc_compute_ms']) > 1000 * float(r['control_period']) for r in rows)
    if events:
        with trial.with_suffix('.touchdowns.csv').open('w') as handle:
            writer = csv.DictWriter(handle, list(events[0]))
            writer.writeheader()
            writer.writerows(events)
        result['detected_touchdowns'] = len(events)
        result['touchdown_abs_time_ms_p50_p95_p99'] = quantiles([abs(e['timing_ms']) for e in events])
        result['estimated_precontact_vz_mps_p50_p95_p99'] = quantiles([e['precontact_vz_mps'] for e in events])
        plot_touchdown(rows, events, trial.with_name(trial.stem + '_touchdown'))
    decisions = [r for r in replans['decisions'] if r.get('remaining') is not None]
    if decisions:
        fig, ax = plt.subplots(figsize=(3.6, 2.6), layout='constrained')
        for accepted, color, marker in [(False, '#9ca4ac', 'x'), (True, '#157285', 'o')]:
            records = [r for r in decisions if (r['outcome'] == 1) == accepted]
            ax.scatter([1000*r['remaining'] for r in records], [1000*r['position_delta'] for r in records],
                       s=9, marker=marker, color=color, alpha=.68,
                       label=('Admitted' if accepted else 'Rejected') + f' ({len(records)})')
        ax.set(xlabel='Remaining swing at decision (ms)', ylabel='Initial position deviation (mm)')
        ax.legend(loc='lower left', bbox_to_anchor=(0, 1.02), ncol=2, fontsize=7)
        save(fig, trial.with_name(trial.stem + '_admission'))
    if replans['applied']:
        applied = replans['applied']
        delays = np.array([1000*(r['applied']-r['activation']) for r in applied])
        errors = np.array([1000*r['position_error'] for r in applied])
        result['activation_delay_ms_p50_p95_p99'] = quantiles(delays)
        result['activation_position_error_mm_p50_p95_p99'] = quantiles(errors)
        fig, ax = plt.subplots(figsize=(3.6, 2.6), layout='constrained')
        ax.scatter([1000*r['remaining'] for r in applied], errors, s=12, color='#157285', alpha=.72)
        ax.set(xlabel='Remaining swing at activation (ms)', ylabel='Foot position mismatch (mm)',
               title=f'{meta["scenario"]}: {len(applied)} applied replans')
        save(fig, trial.with_name(trial.stem + '_replan'))
    if not rows or 'control_period' not in rows[0]:
        print(json.dumps(result, indent=2))
        return
    statuses = np.array([[int(float(r[f'{leg}_safety_status'])) for leg in LEGS] for r in rows]).ravel()
    tubes = np.array([[int(float(r[f'{leg}_tube_status'])) for leg in LEGS] for r in rows]).ravel()
    corrections = np.array([[float(r[f'{leg}_correction']) for leg in LEGS] for r in rows]).ravel()
    residual = finite([r[f'{leg}_wbc_safety_residual'] for r in rows for leg in LEGS])
    clearance = finite([r[f'{leg}_clearance'] for r in rows for leg in LEGS])
    filtered = np.isin(statuses, [1, 13, 15, 16])
    counts = Counter(int(s) for s in statuses if s != 12)
    result.update(safety_counts={STATUS[s]: n for s, n in sorted(counts.items())},
                  control_period_ms_p50_p95_p99=quantiles([1000*float(r['control_period']) for r in rows]),
                  filtered=int(np.count_nonzero(filtered)),
                  tube_enforced=int(np.count_nonzero(filtered & (tubes == 1))),
                  tube_status_counts={STATUS[int(s)]: int(n)
                                      for s, n in sorted(Counter(tubes[filtered]).items())},
                  corrected=int(np.count_nonzero(filtered & (corrections > 1e-5))),
                  invalid_wbc_samples=sum(float(r['valid']) != 1 for r in rows),
                  correction_ms2_p50_p95_p99=quantiles(corrections[filtered]),
                  worst_sampled_wbc_safety_residual=float(min(residual)) if len(residual) else None,
                  min_estimated_corridor_clearance_mm=float(1000*min(clearance)) if len(clearance) else None)
    if counts and counts.keys() != {0}:
        fig, ax = plt.subplots(figsize=(3.8, 2.8), layout='constrained')
        plotted = sorted(counts, key=counts.get)
        bars = ax.barh([STATUS[s] for s in plotted], [counts[s] for s in plotted], color=[
            '#157285' if s in (1, 13, 15, 16) else '#a7afb6' for s in plotted])
        ax.bar_label(bars, padding=3)
        ax.set(xlim=(0, 1.15*max(counts.values())), xlabel='Swing-leg samples',
               title=f'{meta["scenario"]}: safety filter outcome')
        save(fig, trial.with_name(trial.stem + '_safety'))
    trial.with_suffix('.metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
