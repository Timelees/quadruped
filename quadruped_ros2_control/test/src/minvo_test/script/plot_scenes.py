#!/usr/bin/env python3
"""Plot exported production splines; no Python spline or optimizer implementation.
Use system Python for the system Matplotlib 3D extension on the development host.
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import yaml

COLORS = {'reference': '#ec2634', 'Bezier': '#ef9a23', 'MINVO': '#08a532', 'MINVO_geometry': '#2471cb'}
LABELS = {'reference': '参考轨迹', 'Bezier': 'Bezier + 跨越约束', 'MINVO': 'MINVO + 跨越约束',
          'MINVO_geometry': 'MINVO 几何约束'}
SCENE_NAMES = {'single': '单障碍', 'pillars': '连续柱台', 'stairs': '楼梯'}


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def column(data, name):
    return np.array([float(row[name]) for row in data])


def polygon(points):
    points = sorted(set(map(tuple, points)))
    if len(points) < 3:
        return points
    def cross(a, b, c):
        return np.cross(np.subtract(b, a), np.subtract(c, a))
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def curve(ax, data, method, axes=('x', 'z'), label=True):
    group = [r for r in data if r['method'] == method]
    if not group:
        return
    style = '--' if method == 'reference' else '-'
    ax.plot(*(column(group, key) for key in axes), color=COLORS[method], linestyle=style,
            linewidth=1.8, label=LABELS[method] if label else None, zorder=3)
    if axes == ('x', 'z'):
        ax.plot(column(group, 'x')[[0, -1]], column(group, 'z')[[0, -1]], 'o',
                color=COLORS[method], markerfacecolor='white', markersize=4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--data', type=Path, default=root / 'data')
    parser.add_argument('--result', type=Path, default=root / 'result')
    parser.add_argument('--font', type=Path, help='宋体字体文件，例如 simsun.ttc')
    args = parser.parse_args()
    args.result.mkdir(parents=True, exist_ok=True)
    if args.font:
        font_manager.fontManager.addfont(str(args.font))
        family = font_manager.FontProperties(fname=str(args.font)).get_name()
    else:
        installed = {f.name for f in font_manager.fontManager.ttflist}
        family = next((f for f in ['SimSun', '宋体', 'NSimSun', 'Noto Serif CJK SC',
                                  'Noto Serif CJK JP'] if f in installed), None)
        if family is None:
            raise SystemExit('未找到宋体或中文衬线字体，请用 --font 指定字体文件')
        if family not in ['SimSun', '宋体', 'NSimSun']:
            print('未安装微软宋体，使用思源宋体替代；可用 --font 指定 simsun.ttc')
    plt.rcParams.update({'font.family': family, 'svg.fonttype': 'none', 'axes.unicode_minus': False})
    scenes = [p for p in args.data.iterdir() if (p / 'map.csv').exists()]
    if not scenes:
        raise SystemExit('No scene exports. Run minvo_scene CONFIG DATA/SCENE --export-only first.')
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.dpi': 180})
    metrics = []
    for directory in sorted(scenes):
        name = directory.name
        terrain, trajectories, hulls = (rows(directory / f) for f in ['map.csv', 'trajectories.csv', 'hulls.csv'])
        with (directory / 'config.yaml').open() as stream:
            scene_config = yaml.safe_load(stream)
            visualization = scene_config['visualization']
        cmap = LinearSegmentedColormap.from_list('height_colors',
            np.array(visualization['colors']) / 255.0, N=1024)
        xs = np.unique(column(terrain, 'x')); ys = np.unique(column(terrain, 'y'))
        heights = {(float(r['x']), float(r['y'])): float(r['z']) for r in terrain}
        xx, yy = np.meshgrid(xs, ys)
        zz = np.array([[heights[x, y] for x in xs] for y in ys])
        norm = Normalize(*visualization['height_range'])
        methods = [m for m in ['reference', 'Bezier', 'MINVO_geometry', 'MINVO'] if any(r['method'] == m for r in trajectories)]
        if 'MINVO_geometry' not in methods:
            LABELS['Bezier'], LABELS['MINVO'] = 'Bezier 优化', 'MINVO 优化'
        else:
            LABELS['Bezier'], LABELS['MINVO'] = 'Bezier + 跨越约束', 'MINVO + 跨越约束'
        swings = sorted({int(r['swing']) for r in trajectories})
        fig = plt.figure(figsize=(14, 8), constrained_layout=True)
        grid = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
        spatial = fig.add_subplot(grid[:, 0], projection='3d', computed_zorder=False)
        resolution = scene_config['map']['resolution']
        half_cell = resolution / 2
        top_faces, top_colors = [], []
        for row, y in enumerate(ys):
            start = 0
            for stop in range(1, len(xs) + 1):
                if stop < len(xs) and zz[row, stop] == zz[row, start]:
                    continue
                x0, x1, h = xs[start]-half_cell, xs[stop-1]+half_cell, zz[row, start]
                top_faces.append([(x0,y-half_cell,h), (x1,y-half_cell,h),
                                  (x1,y+half_cell,h), (x0,y+half_cell,h)])
                top_colors.append(cmap(norm(h)))
                start = stop
        # A single collection sorts terrain faces together in the 3D view.
        # Keep top faces height colored; render each cliff as horizontal color bands.
        walls = top_faces
        wall_colors = top_colors
        for axis in [0, 1]:
            values = xs if axis == 0 else ys
            for index in range(len(values) - 1):
                low = zz[:, index] if axis == 0 else zz[index, :]
                high = zz[:, index + 1] if axis == 0 else zz[index + 1, :]
                for lane in range(len(low)):
                    if abs(low[lane] - high[lane]) < 1e-9:
                        continue
                    lower, upper = sorted((low[lane], high[lane]))
                    transverse = ys[lane] if axis == 0 else xs[lane]
                    for z0 in np.arange(lower, upper, resolution):
                        z1 = min(z0 + resolution, upper)
                        if axis == 0:
                            face = [((values[index] + values[index + 1]) / 2, transverse-half_cell, z0),
                                    ((values[index] + values[index + 1]) / 2, transverse+half_cell, z0),
                                    ((values[index] + values[index + 1]) / 2, transverse+half_cell, z1),
                                    ((values[index] + values[index + 1]) / 2, transverse-half_cell, z1)]
                        else:
                            face = [(transverse-half_cell, (values[index] + values[index + 1]) / 2, z0),
                                    (transverse+half_cell, (values[index] + values[index + 1]) / 2, z0),
                                    (transverse+half_cell, (values[index] + values[index + 1]) / 2, z1),
                                    (transverse-half_cell, (values[index] + values[index + 1]) / 2, z1)]
                        walls.append(face)
                        wall_colors.append(cmap(norm((z0 + z1) / 2)))
        spatial.add_collection3d(Poly3DCollection(walls, facecolors=wall_colors,
                                                   edgecolors='none', antialiased=False, zorder=1))
        spatial.set(xlim=(xs[0], xs[-1]), ylim=(ys[0], ys[-1]),
                    zlim=(0, max(float(zz.max()), column(trajectories, 'z').max()) * 1.15))
        side = fig.add_subplot(grid[0, 1]); top = fig.add_subplot(grid[1, 1])
        top.pcolormesh(xx, yy, zz, cmap=cmap, norm=norm, shading='nearest')
        nearest_y = ys[np.argmin(np.abs(ys))]
        side.fill_between(xs, 0, [heights[x, nearest_y] for x in xs], color='#91969b', alpha=.6, step='mid')
        for swing in swings:
            group = [r for r in trajectories if int(r['swing']) == swing]
            for method in methods:
                curve(spatial, group, method, ('x', 'y', 'z'), swing == 0)
                curve(side, group, method, label=swing == 0)
                curve(top, group, method, ('x', 'y'), swing == 0)
        spatial.set(xlabel='前进方向 x (m)', ylabel='横向 y (m)', zlabel='高程 z (m)', title='(a) 高程图与足端轨迹')
        spatial.set_box_aspect((2.2, 1, .85)); spatial.view_init(20, -76)
        spatial.set_proj_type('ortho')
        side.set(xlabel='前进方向 x (m)', ylabel='足端高度 z (m)', title='(b) 侧视图与碰撞几何')
        top.set(xlabel='前进方向 x (m)', ylabel='横向 y (m)', title='(c) 俯视高程图')
        top.set_aspect('equal'); side.grid(alpha=.2); side.legend(fontsize=8)
        fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=[spatial, top], label='高程 (m)', shrink=.55)
        fig.suptitle(f'{SCENE_NAMES.get(name, name)}：高程地图与优化轨迹')
        for suffix in ['png', 'svg']:
            fig.savefig(args.result / f'{name}_overview.{suffix}')
        plt.close(fig)

        fig, axes = plt.subplots(len(swings), 2, figsize=(13, max(4, 3.3 * len(swings))), squeeze=False, constrained_layout=True)
        for swing, axrow in zip(swings, axes):
            group = [r for r in trajectories if int(r['swing']) == swing]
            for ax in axrow:
                ax.fill_between(xs, 0, [heights[x, nearest_y] for x in xs], color='#91969b', alpha=.4, step='mid')
            for method in methods:
                curve(axrow[0], group, method)
            # Each optimizer's own curve and own basis, separate from the same-curve comparison.
            for method in ['Bezier', 'MINVO']:
                own = [r for r in hulls if int(r['swing']) == swing and r['method'] == method and r['basis'] == method]
                for span in sorted({r['span'] for r in own}):
                    points = [r for r in own if r['span'] == span]
                    boundary = polygon(np.c_[column(points, 'x'), column(points, 'z')])
                    if len(boundary) >= 3:
                        axrow[0].add_patch(Polygon(boundary, facecolor=COLORS[method], edgecolor=COLORS[method], alpha=.10))
            selected = [r for r in hulls if int(r['swing']) == swing and r['method'] == 'MINVO']
            for basis, color in [('Bezier', '#ef9a23'), ('MINVO', '#08a532')]:
                total = 0
                for span in sorted({r['span'] for r in selected}):
                    points = [r for r in selected if r['span'] == span and r['basis'] == basis]
                    boundary = polygon(np.c_[column(points, 'x'), column(points, 'z')])
                    if len(boundary) < 3:
                        continue
                    p = np.array(boundary)
                    total += abs(np.dot(p[:, 0], np.roll(p[:, 1], -1)) - np.dot(p[:, 1], np.roll(p[:, 0], -1))) / 2
                    axrow[1].add_patch(Polygon(p, facecolor=color, edgecolor=color, alpha=.2))
                    axrow[1].scatter(column(points, 'x'), column(points, 'z'), s=9, c=color)
                metrics.append({'scene': name, 'swing': swing, 'basis': basis, 'xz_hull_area_sum': total})
            curve(axrow[1], group, 'MINVO')
            for ax in axrow:
                allx = column(group, 'x'); ax.set_xlim(allx.min()-.025, allx.max()+.025)
                ax.set(xlabel='x (m)', ylabel='z (m)'); ax.grid(alpha=.2)
            axrow[0].set_title(f'摆动 {swing}：参考轨迹与优化轨迹'); axrow[0].legend(fontsize=8)
            axrow[1].set_title('同一 MINVO 曲线与分段：Bezier 包络 / MINVO 包络')
        fig.savefig(args.result / f'{name}_swings_hulls.png'); fig.savefig(args.result / f'{name}_swings_hulls.svg'); plt.close(fig)

        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, constrained_layout=True)
        for method in methods:
            group = [r for r in trajectories if int(r['swing']) == swings[len(swings)//2] and r['method'] == method]
            for ax, key in zip(axes, ['z', 'vz', 'az']):
                ax.plot(column(group, 't'), column(group, key), color=COLORS[method], label=LABELS[method],
                        linestyle='--' if method == 'reference' else '-')
                ax.set_ylabel({'z':'足端高度 z (m)', 'vz':'垂直速度 (m/s)', 'az':'垂直加速度 (m/s²)'}[key]); ax.grid(alpha=.2)
        axes[0].legend(fontsize=8); axes[-1].set_xlabel('摆动时间 (s)')
        fig.suptitle(f'{SCENE_NAMES.get(name, name)}：摆动 {swings[len(swings)//2]} 的位置、速度与加速度')
        fig.savefig(args.result / f'{name}_kinematics.png'); fig.savefig(args.result / f'{name}_kinematics.svg'); plt.close(fig)

        # Detail panel follows the reference figure layout: actual collision + local hull geometry.
        reference = [r for r in trajectories if r['method'] == 'reference']
        # Independent witness against the nominal boxes, so the drawn sphere visibly
        # intersects an obstacle rather than only a conservative grid interpolation patch.
        radius = scene_config['foot_radius']
        def nominal_margin(row):
            x, y, z = (float(row[k]) for k in ['x', 'y', 'z'])
            distance = min([z + radius] + [math.sqrt(
                max(x0-x, 0, x-x1)**2 + max(y0-y, 0, y-y1)**2 +
                max(z+radius-height, 0)**2)
                for x0, x1, y0, y1, height in scene_config['obstacles']])
            return distance - radius
        witness = min(reference, key=nominal_margin)
        detail_title = '参考碰撞与优化结果' if nominal_margin(witness) < -1e-7 else '参考与优化的接触位置'
        swing = int(witness['swing'])
        group = [r for r in trajectories if int(r['swing']) == swing]
        focus = float(witness['x'])
        fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
        ax.fill_between(xs, 0, [heights[x, nearest_y] for x in xs], color='#91969b', alpha=.5, step='mid')
        for method in methods:
            curve(ax, group, method)
        radius = scene_config['foot_radius']
        for method in ['reference', 'MINVO']:
            samples = [r for r in group if r['method'] == method]
            contact = min(samples, key=lambda r: abs(float(r['t']) - float(witness['t'])))
            ax.add_patch(Circle((float(contact['x']), float(contact['z']) + radius), radius,
                               fill=False, edgecolor=COLORS[method], linewidth=1.8,
                               label=LABELS[method] + '：同一时刻足端轮廓'))
        ax.set_aspect('equal', adjustable='box')
        ax.axvline(focus, color='#ec2634', linewidth=.7, alpha=.4)
        ax.set(xlim=(focus-.12, focus+.12), xlabel='前进方向 x (m)', ylabel='足端高度 z (m)',
               title=f'{SCENE_NAMES.get(name, name)}：摆动 {swing} {detail_title}')
        ax.legend(fontsize=8); ax.grid(alpha=.2)
        fig.savefig(args.result / f'{name}_collision_detail.png'); fig.savefig(args.result / f'{name}_collision_detail.svg'); plt.close(fig)
    with (args.data / 'hull_metrics.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(metrics[0]));writer.writeheader();writer.writerows(metrics)
    print(f'Wrote PNG/SVG figures to {args.result}')


if __name__ == '__main__':
    main()
