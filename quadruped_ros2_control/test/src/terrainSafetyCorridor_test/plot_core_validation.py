#!/usr/bin/env python3
"""Plot deterministic update experiments and measured Gazebo execution diagnostics.

Raw CSV/JSON files are retained. Finite-difference acceleration residuals are
empirical estimates, not certified disturbance bounds. Every trial is a separate
experimental unit; time/leg samples do not form independent repeats.
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

LEGS = ('LF', 'RF', 'LH', 'RH')
COLORS = ('#777777', '#b66d36', '#287b8e')
TRIAL_LABELS = {'guard2_pulse_hocbf': 'HOCBF, bound 0.5',
                'guard2_pulse_full': 'Full, bound 0.5',
                'calibrated_pulse_full': 'Full, bound 35',
                'observed_pulse_off': 'No filter, baseline',
                'observed_pulse_hocbf': 'HOCBF, bound 0.5',
                'observed_pulse_full': 'Full, bound 0.5',
                'observed_calibrated_full': 'Full, bound 35'}


def trial_color(name):
    if 'calibrated' in name:
        return '#76679b'
    if 'hocbf' in name:
        return '#b66d36'
    return '#777777' if name.endswith('_off') else '#287b8e'


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def save(fig, stem):
    from audit_panel_alignment import require_matplotlib_panel_alignment
    require_matplotlib_panel_alignment(fig, json_out=str(stem)+'.alignment.json', strict=True)
    for extension in ('.png', '.pdf', '.svg'):
        fig.savefig(str(stem)+extension, dpi=300)
    plt.close(fig)


def update_scan(directory):
    path = directory/'controlled_update.csv'
    if not path.exists():
        return
    rows = read(path)
    heights = sorted({float(r['height_m']) for r in rows})
    phases = sorted({float(r['phase']) for r in rows})
    shifts = sorted({float(r['state_shift_mm']) for r in rows})
    fig, axes = plt.subplots(1, len(heights), figsize=(7.2, 2.5), sharex=True, sharey=True, layout='constrained')
    for ax, height in zip(axes, heights):
        matrix = np.array([[np.mean([float(r['admitted']) for r in rows
                            if int(r['method']) == 2 and float(r['height_m']) == height
                            and float(r['phase']) == phase and float(r['state_shift_mm']) == shift])
                            for phase in phases] for shift in shifts])
        im = ax.imshow(matrix, origin='lower', aspect='auto', vmin=0, vmax=1, cmap='Blues')
        ax.set(xticks=range(len(phases)), xticklabels=[f'{p:.2f}' for p in phases],
               yticks=range(len(shifts)), yticklabels=[f'{s:g}' for s in shifts],
               xlabel='Update phase', title=f'Revealed obstacle: {height*100:.0f} cm')
    axes[0].set_ylabel('State offset (mm)')
    fig.colorbar(im, ax=list(axes), label='Admission fraction over 3 delay values', shrink=.85)
    save(fig, directory/'controlled_admission')
    summary = {}
    fig, ax = plt.subplots(figsize=(3.8, 2.8), layout='constrained')
    for method, label, color in zip(range(3), ('No update', 'Direct replacement', 'Splice + admission'), COLORS):
        subset = [r for r in rows if int(r['method']) == method]
        accepted = [r for r in subset if int(r['admitted'])]
        jumps = np.sort([float(r['splice_mm']) for r in accepted])
        if len(jumps):
            ax.step(jumps, np.arange(1, len(jumps)+1)/len(jumps), where='post', color=color, label=label)
        summary[label] = dict(cases=len(subset), admitted=len(accepted),
            numerically_safe=sum(int(r['reference_safe']) for r in accepted),
            max_splice_mm=max(jumps) if len(jumps) else None,
            certified_scope='noncontact phase [0.25,0.75]; geometry oracle step 0.1 ms')
    ax.set(xlabel='Position mismatch at update (mm)', ylabel='Cumulative fraction', ylim=(0, 1.03))
    ax.legend(loc='lower right', fontsize=7)
    save(fig, directory/'controlled_splice')
    (directory/'controlled_summary.json').write_text(json.dumps(summary, indent=2)+'\n')


def execution(directory, trials, figure_trials=None):
    summary = {}
    residuals = []
    coverages = []
    configured_bounds = set()
    for path in sorted(directory.glob('trial_*_wbc.csv')):
        if trials and path.stem.removesuffix('_wbc') not in trials:
            continue
        rows = read(path)
        if not rows or 'LF_command_accel_x' not in rows[0]:
            continue
        task = directory/(path.stem.removesuffix('_wbc')+'.task.info')
        disturbance_bound = float(re.search(r'^\s*safetyFilterDisturbanceBound\s+(\S+)', task.read_text(), re.M).group(1))
        correction_budget = float(re.search(r'^\s*safetyFilterMaxCorrection\s+(\S+)', task.read_text(), re.M).group(1))
        time = np.array([float(r['sim_time']) for r in rows])
        mode = np.array([int(float(r['mode'])) for r in rows])
        dt = np.diff(time)
        stats, samples, active_samples = {}, [], []
        for leg, bit in zip(LEGS, (8, 4, 2, 1)):
            status = np.array([int(float(r[f'{leg}_safety_status'])) for r in rows])
            swing = (mode & bit) == 0
            force = np.array([float(r[f'sensor_{leg}']) for r in rows])
            valid = (dt > .003) & (dt < .0075) & swing[:-1] & swing[1:] & (force[:-1] < 3) & (force[1:] < 3)
            velocity = np.array([[float(r[f'{leg}_v_{axis}']) for axis in 'xyz'] for r in rows])
            command = np.array([[float(r[f'{leg}_command_accel_{axis}']) for axis in 'xyz'] for r in rows])
            estimate = np.diff(velocity, axis=0)/dt[:, None]-command[:-1]
            norm = np.linalg.norm(estimate, axis=1)
            valid &= np.isfinite(norm)
            samples.extend(norm[valid])
            joint = np.isin(status, (1, 13, 15, 16))
            active_samples.extend(norm[valid & joint[:-1] & joint[1:]])
            requested = swing & np.isfinite([float(r[f'{leg}_clearance']) for r in rows])
            residual = np.array([float(r[f'{leg}_wbc_safety_residual']) for r in rows])
            correction = np.array([float(r[f'{leg}_correction']) for r in rows])
            def duration(mask):
                longest = current = 0.
                for gap, selected in zip(np.r_[dt, 0.], mask):
                    current = current+gap if selected and 0 < gap < .01 else 0.
                    longest = max(longest, current)
                return longest
            stats[leg] = dict(swing_samples=int(swing.sum()), joint_samples=int(np.sum(swing & joint)),
                              tube_joint_samples=int(np.sum(swing & np.isin(status, (1, 13)) &
                                  np.array([int(float(r[f'{leg}_tube_status'])) == 1 for r in rows]))),
                              fallback_samples=int(np.sum(swing & (status == 11))),
                              residual_intervals=int(valid.sum()), requested_samples=int(requested.sum()),
                              active_fraction_of_requested=float(np.mean(joint[requested])) if requested.any() else None,
                              max_final_constraint_violation_mps2=float(max(0., -np.min(residual[joint]))) if joint.any() else None,
                              corrected_joint_samples=int(np.sum(joint & (correction > 1e-6))),
                              longest_joint_interval_s=duration(swing & joint),
                              longest_requested_unavailable_interval_s=duration(requested & ~joint),
                              status_counts={str(s): int(np.sum(swing & (status == s))) for s in np.unique(status[swing])})
            if f'{leg}_baseline_accel_x' in rows[0]:
                baseline = np.array([[float(r[f'{leg}_baseline_accel_{axis}']) for axis in 'xyz'] for r in rows])
                deviation = np.max(np.abs(command-baseline), axis=1)
                selected = deviation[swing & joint & np.isfinite(deviation)]
                stats[leg]['baseline_deviation_max_abs_axis_mps2'] = float(selected.max()) if len(selected) else None
                stats[leg]['baseline_deviation_P50_P95_P99_mps2'] = np.percentile(selected, [50, 95, 99]).tolist() if len(selected) else None
                stats[leg]['changed_from_baseline_joint_samples'] = int(np.sum(selected > 1e-6))
                stats[leg]['baseline_budget_mps2'] = correction_budget
                stats[leg]['baseline_budget_violations'] = int(np.sum(selected > correction_budget+1e-5))
        finite = np.array(samples)
        stats['empirical_residual_mps2'] = dict(intervals=len(samples),
            quantiles_P50_P95_P99=np.percentile(finite, [50, 95, 99]).tolist() if len(finite) else None,
            maximum=float(finite.max()) if len(finite) else None,
            configured_bound=disturbance_bound,
            fraction_above_configured_bound=float(np.mean(finite > disturbance_bound)) if len(finite) else None,
            definition='forward velocity difference minus previous command; no contact force above 3 N; adjacent swing samples 3-7.5 ms apart')
        finite_active = np.array(active_samples)
        stats['active_empirical_residual_mps2'] = dict(intervals=len(active_samples),
            quantiles_P50_P95_P99=np.percentile(finite_active, [50, 95, 99]).tolist() if len(finite_active) else None,
            maximum=float(finite_active.max()) if len(finite_active) else None,
            configured_bound=disturbance_bound,
            fraction_above_configured_bound=float(np.mean(finite_active > disturbance_bound)) if len(finite_active) else None,
            definition='same estimator, restricted to intervals with successful joint constraints at both endpoints')
        stats['observed_interval_ms'] = np.percentile(dt*1000, [50, 95, 99]).tolist()
        summary[path.stem] = stats
        if not figure_trials or path.stem.removesuffix('_wbc') in figure_trials:
            name = path.stem.removesuffix('_wbc').removeprefix('trial_')
            residuals.append((name, finite_active))
            coverages.append((name, sum(stats[leg]['joint_samples'] for leg in LEGS),
                              sum(stats[leg]['swing_samples'] for leg in LEGS)))
            configured_bounds.add(disturbance_bound)
    if residuals:
        if figure_trials:
            order = {name.removeprefix('trial_'): i for i, name in enumerate(figure_trials)}
            residuals.sort(key=lambda item: order[item[0]])
            coverages.sort(key=lambda item: order[item[0]])
        fig, ax = plt.subplots(figsize=(4.2, 3.2), layout='constrained')
        for name, values in residuals:
            values = np.sort(values)
            if len(values):
                ax.step(np.maximum(values, 1e-6), np.arange(1, len(values)+1)/len(values), where='post',
                        color=trial_color(name), label=TRIAL_LABELS.get(name, name.replace('_', ' ')))
        for bound in sorted(configured_bounds):
            ax.axvline(bound, color='0.4', ls='--', lw=.8, label=f'Configured {bound:g}')
        ax.set(xscale='log', xlabel='Residual during joint execution (m/s²)', ylabel='Cumulative fraction', ylim=(0, 1.03))
        ax.legend(fontsize=7, loc='lower center', bbox_to_anchor=(.5, 1.02), ncol=2, frameon=False)
        save(fig, directory/'execution_residual')
        fig, ax = plt.subplots(figsize=(7.2, 2.8), layout='constrained')
        values = [100*joint/total if total else 0 for _, joint, total in coverages]
        ax.bar(range(len(values)), values, color=[trial_color(name) for name, _, _ in coverages], width=.55)
        for i, (value, (_, joint, total)) in enumerate(zip(values, coverages)):
            ax.annotate(f'{value:.1f}%\n{joint}/{total}', (i, value), xytext=(0, 4), textcoords='offset points',
                        ha='center', va='bottom', fontsize=7)
        ax.set(xticks=range(len(values)),
               xticklabels=[TRIAL_LABELS.get(name, name.replace('_', ' ')).replace(', ', '\n')
                            for name, _, _ in coverages],
               ylabel='Joint samples / all swing samples (%)', ylim=(0, max(1., max(values)*1.3)))
        save(fig, directory/'execution_coverage')
    (directory/'execution_summary.json').write_text(json.dumps(summary, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--audit-tools', type=Path, required=True)
    parser.add_argument('--trials', nargs='+', help='Explicit trial stems; separates implementation versions, all raw files retained')
    parser.add_argument('--figure-trials', nargs='+', help='Residual curves to display; statistics still retain every selected trial')
    args = parser.parse_args()
    sys.path.insert(0, str(args.audit_tools))
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 8, 'axes.spines.top': False,
                         'axes.spines.right': False, 'pdf.fonttype': 42, 'svg.fonttype': 'none'})
    update_scan(args.directory)
    execution(args.directory, args.trials, args.figure_trials)


if __name__ == '__main__':
    main()
