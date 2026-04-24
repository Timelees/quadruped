#!/usr/bin/env python3
"""
Plot CSV data in logs/data grouped by filename prefix (before first underscore).
Saves figures to logs/plot_fig/*.png

Usage: python3 plot_data.py
"""
import os
import csv
import math
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

ROOT = os.path.dirname(__file__)
DATA_DIR = os.path.join(ROOT, 'data')
OUT_DIR = os.path.join(ROOT, 'plot_fig')


def read_csv(path):
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            return [], []
        rows = [row for row in reader if row]
    # convert to float where possible, keep as np.array of floats
    if not rows:
        return header, np.empty((0, len(header)))
    arr = np.array([[float(x) for x in row] for row in rows], dtype=float)
    return header, arr


def group_files(files):
    groups = defaultdict(list)
    for f in files:
        if not f.lower().endswith('.csv'):
            continue
        name = os.path.basename(f)
        if '_' in name:
            prefix = name.split('_', 1)[0]
        else:
            prefix = os.path.splitext(name)[0]
        groups[prefix].append(f)
    return groups


def plot_group(prefix, files):
    # One figure per prefix. Each file -> one subplot (stacked vertically)
    n = len(files)
    if n == 0:
        return
    fig, axes = plt.subplots(n, 1, figsize=(8, 3 * n), squeeze=False)
    axes = axes.flatten()
    for idx, f in enumerate(sorted(files)):
        header, data = read_csv(f)
        ax = axes[idx]
        ax.set_title(os.path.basename(f))
        if data.size == 0:
            ax.text(0.5, 0.5, 'empty', ha='center', va='center')
            continue
        cols = data.shape[1]
        x = np.arange(data.shape[0])
        # choose colors from matplotlib colormap
        cmap = plt.get_cmap('tab10')
        for c in range(cols):
            color = cmap(c % 10)
            ax.plot(x, data[:, c], label=(header[c] if c < len(header) else f'c{c}'), color=color)
        ax.grid(True)
        ax.legend(loc='best', fontsize='small')
        ax.set_xlabel('sample')
    fig.tight_layout()
    out_path = os.path.join(OUT_DIR, f"{prefix}.png")
    fig.savefig(out_path)
    plt.close(fig)
    print('Saved', out_path)


def main():
    if not os.path.isdir(DATA_DIR):
        print('Data dir not found:', DATA_DIR)
        return
    os.makedirs(OUT_DIR, exist_ok=True)

    files = [os.path.join(DATA_DIR, f) for f in os.listdir(DATA_DIR) if f.lower().endswith('.csv')]
    groups = group_files(files)
    if not groups:
        print('No csv files found in', DATA_DIR)
        return
    for prefix, fls in groups.items():
        plot_group(prefix, fls)


if __name__ == '__main__':
    main()
