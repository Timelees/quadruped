#!/usr/bin/env python3
"""One figure per terrain, using production-node CSV exports (schema 2).

Displayed coordinates are CONTACT REFERENCES (sphere centre minus foot radius). Collision flags use the complete sphere,
not just centre-versus-height. Corridors apply only to exported non-contact cells.
"""
import argparse
from pathlib import Path
import shutil
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

SCENES = ('flat', 'bar', 'pillars', 'stairs')
COLORS = dict(ref='#C74343', opt='#267699', geom_lower='#D18A28',
              geom_upper='#72777D', plan_lower='#008C33', plan_upper='#8064A2',
              sides='#58A9B6')


def load(path):
    return np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True, encoding=None))


def section_polygon(planes, offset, y, bounds):
    """Clip an XZ rectangle by ALL exported half-spaces at fixed y."""
    xmin, xmax, zmin, zmax = bounds
    polygon = np.array([[xmin, zmin], [xmax, zmin], [xmax, zmax], [xmin, zmax]])
    for plane in planes:
        if len(polygon) == 0:
            break
        normal = np.array([plane['ax'], plane['az']])
        bound = plane[offset] - plane['ay'] * y
        clipped = []
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            da, db = normal @ a - bound, normal @ b - bound
            if da <= 1e-12:
                clipped.append(a)
            if (da <= 0) != (db <= 0):
                clipped.append(a + da / (da - db) * (b - a))
        polygon = np.asarray(clipped).reshape(-1, 2)
    return polygon


def plot_scene(data, output, name, *, require_schema=True, audit_tools=None):
    data, output = Path(data), Path(output)
    manifest = data / f'{name}_swings.csv'
    stems = list(np.atleast_1d(np.genfromtxt(manifest, delimiter=',', names=True, dtype=None, encoding='utf8'))['stem']) if manifest.exists() else [name]
    plane_parts, trajectory_parts, vertex_parts = [], [], []
    intervals = []
    local_intervals = []
    radius = None
    for swing, stem in enumerate(stems):
        planes = load(data / f'{stem}_planes.csv')
        trajectory = load(data / f'{stem}_trajectory.csv')
        metrics_path = data / f'{stem}_metrics.csv'
        if not metrics_path.exists():
            raise ValueError('Legacy exports: rerun the updated terrain_corridor_test for schema 2')
        metrics = load(metrics_path)
        if len(metrics) != 1 or metrics['schema'][0] != 2:
            raise ValueError('Re-export with the updated terrain_corridor_test (schema 2)')
        if require_schema:
            if name != 'flat' and swing == 0 and metrics['reference_min_clearance'][0] >= -1e-4:
                raise ValueError(f'{stem}: no measured nominal collision')
            if metrics['optimized_min_clearance'][0] < -1e-8 or metrics['corridor_max_violation'][0] > 1e-9:
                raise ValueError(f'{stem}: optimized path failed verification')
        radius = float(metrics['foot_radius'][0])
        # Translate BOTH curves and every half-space: p_c=p_ref+r*e_z.
        # a.p_c<=b becomes a.p_ref<=b-a_z*r. Terrain stays unchanged.
        trajectory['ref_z'] -= radius
        trajectory['opt_z'] -= radius
        for offset in ('b_geom', 'b_plan', 'b_qp'):
            planes[offset] -= planes['az'] * radius
        local_intervals.append((planes['start'].min(), planes['end'].max()))
        intervals.append((swing + planes['start'].min(), swing + planes['end'].max()))
        planes['start'] += swing
        planes['end'] += swing
        planes['cell'] += swing * 10000
        trajectory['phase'] += swing
        trajectory['time'] += swing * float(metrics['duration'][0])
        vertices = np.atleast_1d(np.genfromtxt(data / f'{stem}_vertices.csv', delimiter=',', names=True, dtype=None, encoding='utf8'))
        vertices['z'] -= radius
        vertex_parts.append(vertices)
        plane_parts.append(planes)
        trajectory_parts.append(trajectory if swing == 0 else trajectory[1:])
    planes = np.concatenate(plane_parts)
    trajectory = np.concatenate(trajectory_parts)
    vertices = np.concatenate(vertex_parts)
    terrain = load(data / f'{stems[0]}_terrain.csv')
    if not np.all(np.diff(trajectory['phase']) > 0):
        raise ValueError('Phase grid must increase')
    if np.ptp(trajectory['ref_y']) > 1e-9 or np.ptp(trajectory['opt_y']) > 1e-9:
        raise ValueError('XZ section requires a constant-y swing')
    y = trajectory['ref_y'][0]
    xmin = min(trajectory['ref_x'].min(), trajectory['opt_x'].min()) - .055
    xmax = max(trajectory['ref_x'].max(), trajectory['opt_x'].max()) + .055
    # Limits derive from exported cell vertices, never a hard-coded height.
    zmax = max(vertices['z'].max(), trajectory['opt_z'].max(), trajectory['ref_z'].max()) + .015
    bounds = (xmin, xmax, -.01, zmax)
    fig, axs = plt.subplots(1, 2, figsize=(9, 4.5))
    for cell in np.unique(planes['cell']):
        p = planes[planes['cell'] == cell]
        phase = np.array([p['start'][0], p['end'][0]])
        for offset, prefix in [('b_geom', 'geom'), ('b_plan', 'plan')]:
            polygon = section_polygon(p, offset, y, bounds)
            if len(polygon) >= 3:
                axs[0].fill(polygon[:, 0], polygon[:, 1], color=COLORS['sides'], alpha=.025, lw=0)
                for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                    midpoint = .5 * (a + b)
                    active = p[np.abs(p['ax'] * midpoint[0] + p['ay'] * y + p['az'] * midpoint[1] - p[offset]) < 1e-8]
                    if not len(active):  # artificial plotting-window edge
                        continue
                    az = active[0]['az']
                    color = COLORS[prefix + ('_lower' if az < -.5 else '_upper')] if abs(az) > .5 else COLORS['sides']
                    axs[0].plot([a[0], b[0]], [a[1], b[1]], color=color,
                                lw=.8 if prefix == 'plan' else .55, alpha=.8 if prefix == 'plan' else .55)
            for lower in (True, False):
                faces = p[p['az'] < -.5] if lower else p[p['az'] > .5]
                if not len(faces):
                    raise ValueError('Unbounded vertical cell')
                heights = (faces[offset] - faces['ay'] * y) / faces['az']
                height = max(heights) if lower else min(heights)
                axs[1].plot(phase, [height, height], color=COLORS[prefix + ('_lower' if lower else '_upper')], lw=.9)
    # Terrain section is taken from the exported raster, including grid quantization.
    nearest_y = np.unique(terrain['y'])[np.argmin(abs(np.unique(terrain['y']) - y))]
    ground = terrain[terrain['y'] == nearest_y]
    ground = np.sort(ground, order='x')
    resolution = np.min(np.diff(np.unique(ground['x'])))
    edges = np.r_[ground['x'] - resolution / 2, ground['x'][-1] + resolution / 2]
    axs[0].stairs(ground['z'], edges, baseline=-.01, fill=True, color='#B5B6B8', alpha=.8, zorder=1)
    for ax, refx, optx in [(axs[0], trajectory['ref_x'], trajectory['opt_x']),
                           (axs[1], trajectory['phase'], trajectory['phase'])]:
        ax.plot(refx, trajectory['ref_z'], color=COLORS['ref'], ls='--', lw=1.6, zorder=7)
        ax.plot(optx, trajectory['opt_z'], color=COLORS['opt'], lw=1.8, zorder=6)
        if 'ref_clearance' in trajectory.dtype.names:
            collision = np.flatnonzero(trajectory['ref_clearance'] < -1e-8)[::35]
            ax.scatter(refx[collision], trajectory['ref_z'][collision], marker='x', s=18, color=COLORS['ref'], zorder=7)
        ax.set(ylim=(-.01, zmax), ylabel='Foot reference height (m)')
        ax.grid(alpha=.12)
    axs[0].set(xlim=(xmin, xmax), xlabel='Forward position x (m)', title=f'{name.capitalize()}: terrain and XZ section')
    axs[1].set(xlim=(0, len(stems)), xlabel='Swing index + normalized phase', title='Assigned cell bounds versus swing phase')
    for swing, (start, end) in enumerate(intervals):
        for lo, hi in [(swing, start), (end, swing + 1)]:
            axs[1].axvspan(lo, hi, color='#AAAAAA', alpha=.10)
        if swing:
            axs[1].axvline(swing, color='#72777D', ls=':', lw=.6)
    # Mark actual support contacts, without the foot-centre height offset.
    for stem in stems:
        contact = load(data / f'{stem}_trajectory.csv')
        axs[0].scatter(contact['ref_x'][[0, -1]], contact['ref_z'][[0, -1]] - radius,
                       color='#222222', s=12, zorder=9)
    labels = [('ref', 'Nominal swing (before terrain correction)', '--'), ('opt', 'Corridor + MINVO optimized swing', '-'),
              ('geom_lower', 'Geometric lower boundary', '-'), ('geom_upper', 'Geometric upper boundary', '-'),
              ('plan_lower', 'Planning lower boundary', '-'), ('plan_upper', 'Planning upper boundary', '-'),
              ('sides', 'Longitudinal section boundary', '-')]
    handles = [Line2D([], [], color=COLORS[k], ls=style, label=label) for k, label, style in labels]
    handles += [Patch(facecolor='#B5B6B8', label='Input terrain slice'),
                Line2D([], [], color=COLORS['ref'], marker='x', ls='', label='Nominal foot sphere intersects terrain')]
    if not require_schema:
        fig.text(.5, .98, 'LEGACY DATA - layout preview only', ha='center', fontsize=8, color=COLORS['ref'])
    fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=7, frameon=False, bbox_to_anchor=(.5, .05))
    phase_note = ', '.join(f'[{lo:.2f}, {hi:.2f}]' for lo, hi in sorted(set(local_intervals)))
    note = (f'Contact-reference coordinates; corridor local phase {phase_note}. '
            + ('Flat terrain is the collision-free baseline.' if name == 'flat' else 'Crosses denote full-sphere collisions.'))
    fig.text(.5, .015, note, ha='center', fontsize=7)
    fig.subplots_adjust(left=.08, right=.98, bottom=.32, top=.90, wspace=.27)
    output.mkdir(parents=True, exist_ok=True)
    for label, ax in zip(('a', 'b'), axs):
        ax.text(-.12, 1.08, label, transform=ax.transAxes, fontweight='bold')
    fig.canvas.draw()
    if audit_tools:
        sys.path.insert(0, str(audit_tools))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(
            fig, json_out=output / f'corridor_constraint_{name}.alignment.json',
            tolerance_pt=1.5, gutter_tolerance_pt=1.5, strict=True)
    stem = output / f'corridor_constraint_{name}'
    fig.savefig(stem.with_suffix('.png'), dpi=600)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    if name == 'bar':
        for ext in ('png', 'pdf', 'svg'):
            shutil.copyfile(stem.with_suffix('.' + ext), output / f'corridor_constraint.{ext}')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--audit-tools', type=Path)
    parser.add_argument('--scene', choices=(*SCENES, 'all'), default='all')
    args = parser.parse_args()
    plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 8, 'svg.fonttype': 'none', 'pdf.fonttype': 42})
    for name in SCENES if args.scene == 'all' else (args.scene,):
        plot_scene(args.data, args.output, name, audit_tools=args.audit_tools)


if __name__ == '__main__':
    main()
