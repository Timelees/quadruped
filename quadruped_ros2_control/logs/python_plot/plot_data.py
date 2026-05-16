#!/usr/bin/env python3
"""
Plot CSV data in logs/data grouped by filename prefix (before first underscore).
Saves figures to logs/plot_fig/*.png

Usage: python3 plot_data.py
"""
import os
import csv
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(LOGS_DIR, 'data')
OUT_DIR = os.path.join(LOGS_DIR, 'plot_fig')


def read_csv(path):
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            header = [h.strip() for h in next(reader)]
        except StopIteration:
            return [], np.empty((0, 0))
        ncols = len(header)
        parsed = []
        for row in reader:
            if not row:
                continue
            row = [c.strip() for c in row]
            if all(c == '' for c in row):
                continue
            if len(row) < ncols:
                row = row + [''] * (ncols - len(row))
            elif len(row) > ncols:
                row = row[:ncols]
            try:
                parsed.append([float(c) if c != '' else float('nan') for c in row])
            except ValueError:
                continue
    if not parsed:
        return header, np.empty((0, ncols))
    return header, np.array(parsed, dtype=float)


def split_time_axis(header, data):
    """Use time column as x-axis; return (x, plot_headers, plot_data)."""
    if data.size == 0 or not header:
        return np.arange(data.shape[0]), header, data

    time_idx = next((i for i, h in enumerate(header) if h.strip().lower() == 'time'), None)
    if time_idx is None:
        return np.arange(data.shape[0]), header, data

    x = data[:, time_idx]
    plot_cols = [i for i in range(data.shape[1]) if i != time_idx]
    plot_header = [header[i] for i in plot_cols]
    plot_data = data[:, plot_cols] if plot_cols else np.empty((data.shape[0], 0))
    return x, plot_header, plot_data


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
        x, plot_header, plot_data = split_time_axis(header, data)
        if plot_data.size == 0:
            ax.text(0.5, 0.5, 'no data columns', ha='center', va='center')
            continue
        cols = plot_data.shape[1]
        cmap = plt.get_cmap('tab10')
        for c in range(cols):
            color = cmap(c % 10)
            label = plot_header[c] if c < len(plot_header) else f'c{c}'
            ax.plot(x, plot_data[:, c], label=label, color=color)
        ax.grid(True)
        ax.legend(loc='best', fontsize='small')
        ax.set_xlabel('time' if any(h.strip().lower() == 'time' for h in header) else 'sample')
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
