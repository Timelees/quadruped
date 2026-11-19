#!/usr/bin/env python3
"""Plot one leg's measured contract margins and actual WBC execution intervals.

The requested window is stated in the title and saved as metadata. Complete
trial statistics are produced separately by plot_core_validation.py.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--leg', choices=['LF', 'RF', 'LH', 'RH'], default='LF')
    parser.add_argument('--window', nargs=2, type=float, required=True, metavar=('START', 'END'))
    parser.add_argument('--audit-tools', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.audit_tools))
    from audit_panel_alignment import require_matplotlib_panel_alignment
    prefix = args.trial.with_suffix('')
    with Path(str(prefix)+'_wbc.csv').open() as stream:
        complete = list(csv.DictReader(stream))
    rows = [row for row in complete if args.window[0] <= float(row['sim_time']) <= args.window[1]]
    if not rows:
        raise ValueError('Empty requested time window')
    config = args.trial.with_suffix('.task.info').read_text()
    def parameter(name):
        return float(re.search(r'^\s*'+name+r'\s+(\S+)', config, re.M).group(1))
    leg = args.leg
    def series(field):
        return np.array([float(row[field]) for row in rows])
    time = series('sim_time')
    position_error = np.array([series(f'{leg}_p_{a}')-series(f'{leg}_pd_{a}') for a in 'xyz']).T
    velocity_error = np.array([series(f'{leg}_v_{a}')-series(f'{leg}_vd_{a}') for a in 'xyz']).T
    known = np.isfinite(series(leg+'_tube_slack'))
    margin = [1-np.max(np.abs(position_error), axis=1)/parameter('safetyFilterPositionBound'),
              1-np.max(np.abs(velocity_error), axis=1)/parameter('safetyFilterVelocityBound'),
              1-np.max(parameter('safetyFilterCouplingGain')*np.abs(position_error)+np.abs(velocity_error), axis=1)/parameter('safetyFilterCouplingBound')]
    status = series(leg+'_safety_status').astype(int)
    joint = np.isin(status, (1, 13))
    command = np.array([series(f'{leg}_command_accel_{a}') for a in 'xyz']).T
    nominal = np.array([series(f'{leg}_nominal_accel_{a}') for a in 'xyz']).T
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 8, 'pdf.fonttype': 42,
                         'svg.fonttype': 'none', 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(4, 1, figsize=(7.2, 5.4), sharex=True)
    fig.subplots_adjust(left=.15, right=.98, bottom=.09, top=.94, hspace=.65)
    for values, name, color in zip(margin, ('Position', 'Velocity', 'Coupled'), ('#287b8e', '#b66d36', '#76679b')):
        axes[0].plot(time, np.where(known, values, np.nan), label=name, color=color, lw=.9)
    axes[0].axhline(0, color='.4', ls='--', lw=.7)
    axes[0].set_ylabel('Normalized tube\nface margin')
    axes[0].legend(ncol=3, loc='lower left', bbox_to_anchor=(0, 1.03), fontsize=7, frameon=False)
    axes[1].plot(time, series(leg+'_psi1'), color='#287b8e', lw=.9)
    axes[1].axhline(0, color='.4', ls='--', lw=.7)
    axes[1].set_ylabel('HOCBF entry\nmargin (m/s)')
    axes[2].plot(time, np.where(joint, series(leg+'_correction'), np.nan), label='Foot QP proposal', color='#b66d36', lw=.9)
    axes[2].plot(time, np.where(joint, np.linalg.norm(command-nominal, axis=1), np.nan), label='Final WBC', color='#287b8e', lw=.9)
    axes[2].set_ylabel('Acceleration\nchange (m/s²)')
    axes[2].legend(ncol=2, loc='lower left', bbox_to_anchor=(0, 1.03), fontsize=7, frameon=False)
    axes[3].step(time, np.where(status == 13, 2, joint.astype(int)), where='post', color='#287b8e', lw=1.)
    axes[3].set(yticks=[0, 1, 2], yticklabels=['Baseline', 'Joint', 'Landing'], ylim=(-.2, 2.2),
                xlabel='Controller time (s)', ylabel='Execution')
    for letter, ax in zip('abcd', axes):
        ax.annotate(letter, (0, 1), xycoords='axes fraction', xytext=(-26, 4), textcoords='offset points',
                    fontsize=9, fontweight='bold', va='bottom')
    stem = args.trial.with_suffix('.'+leg+'_contract')
    require_matplotlib_panel_alignment(fig, json_out=str(stem)+'.alignment.json', strict=True)
    for extension in ('.pdf', '.svg', '.png'):
        fig.savefig(str(stem)+extension, dpi=300)
    plt.close(fig)
    Path(str(stem)+'.metadata.json').write_text(json.dumps(dict(trial=str(args.trial), leg=leg,
        time_window_controller_seconds=args.window, complete_samples=len(complete), plotted_samples=len(rows),
        normalized_margin='minimum over all signed faces within position, velocity, and coupled groups; positive is inside',
        excluded_from_tube_lines='samples without a finite tube diagnostic; baseline execution is retained in panel d'), indent=2)+'\n')


if __name__ == '__main__':
    main()
