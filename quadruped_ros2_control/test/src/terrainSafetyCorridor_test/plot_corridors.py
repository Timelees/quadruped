#!/usr/bin/env python3
"""Render exported, measured 3D corridor cells, no geometry reconstruction from sketches."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

parser = argparse.ArgumentParser()
parser.add_argument('directory', type=Path)
args = parser.parse_args()
fig = plt.figure(figsize=(15, 10))
colors = ['#2b8cbe', '#e08214', '#7b3294', '#238b45']
for panel, name in enumerate(['flat', 'bar', 'pillars', 'stairs']):
    ax = fig.add_subplot(2, 2, panel + 1, projection='3d')
    vertices = np.genfromtxt(args.directory / f'{name}_vertices.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
    trajectory = np.genfromtxt(args.directory / f'{name}_trajectory.csv', delimiter=',', names=True)
    terrain = np.genfromtxt(args.directory / f'{name}_terrain.csv', delimiter=',', names=True)
    x, y = np.unique(terrain['x']), np.unique(terrain['y'])
    xx, yy = np.meshgrid(x, y, indexing='ij')
    zz = np.zeros_like(xx)
    for p in terrain:
        zz[np.searchsorted(x, p['x']), np.searchsorted(y, p['y'])] = p['z']
    ax.plot_surface(xx, yy, zz, color='#aaa9a5', alpha=0.75, linewidth=0, zorder=0)
    # Every second cell is drawn to keep overlapping exact polyhedra readable.
    for cell in np.unique(vertices['cell'])[::2]:
        v = vertices[(vertices['cell'] == cell) & (vertices['type'] == 'plan')]
        xyz = np.column_stack([v['x'], v['y'], v['z']])
        hull = ConvexHull(xyz)
        poly = Poly3DCollection(xyz[hull.simplices], facecolor=colors[cell % 4], edgecolor=colors[cell % 4], alpha=0.08, linewidth=0.4)
        ax.add_collection3d(poly)
    ax.plot(trajectory['opt_x'], trajectory['opt_y'], trajectory['opt_z'], color='#c52b2b', linewidth=2.8, label='Optimized sphere centre')
    ax.plot(trajectory['ref_x'], trajectory['ref_y'], trajectory['ref_z'], color='#222', linestyle='--', linewidth=1, label='Reference sphere centre')
    ax.set(xlim=(-0.07, 0.57), ylim=(-0.25, 0.25), zlim=(0, 0.42), xlabel='x [m]', ylabel='y [m]', zlabel='z [m]', title=name)
    ax.set_box_aspect((1.3, 1, 0.85))
    ax.view_init(elev=20, azim=-65)
    ax.legend(fontsize=7, loc='upper right')
fig.suptitle('Measured 3D planning corridors and MINVO trajectory — non-contact phase [0.1, 0.9]', fontsize=14)
fig.tight_layout()
fig.savefig(args.directory / 'corridor_validation.png', dpi=180)
print(args.directory / 'corridor_validation.png')
