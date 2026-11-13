#!/usr/bin/env python3
"""Compare three local hulls on ONE exported production B-spline, without spline reimplementation."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon
import numpy as np
import yaml

from plot_scenes import rows, column, polygon

BASES = [('B-spline', '#5966ad'), ('Bezier', '#e99528'), ('MINVO', '#13975b')]


def area(points):
    boundary = np.asarray(polygon(points))
    if len(boundary) < 3:
        return 0.0
    return abs(np.dot(boundary[:, 0], np.roll(boundary[:, 1], 1)) -
               np.dot(boundary[:, 1], np.roll(boundary[:, 0], 1))) / 2


def configure_font(path):
    if path:
        font_manager.fontManager.addfont(str(path))
        family = font_manager.FontProperties(fname=str(path)).get_name()
    else:
        installed = {f.name for f in font_manager.fontManager.ttflist}
        family = next((f for f in ['SimSun', '宋体', 'Noto Serif CJK SC', 'Noto Serif CJK JP']
                       if f in installed), None)
        if family is None:
            raise SystemExit('请用 --font 指定宋体字体文件')
        if family not in ['SimSun', '宋体']:
            print('使用思源宋体；可用 --font 指定微软宋体文件')
    plt.rcParams.update({'font.family': family, 'axes.unicode_minus': False,
                         'svg.fonttype': 'none', 'font.size': 11, 'savefig.dpi': 180})


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=['single'], default='single')
    parser.add_argument('--swing', type=int, default=0)
    parser.add_argument('--method', choices=['reference', 'Bezier', 'MINVO', 'MINVO_geometry'], default='MINVO')
    parser.add_argument('--span', type=int, help='原始多项式段编号；默认选择 MINVO/Bezier 面积比最小的非退化段')
    parser.add_argument('--data', type=Path, default=root / 'data')
    parser.add_argument('--result', type=Path, default=root / 'result')
    parser.add_argument('--font', type=Path)
    args = parser.parse_args()
    configure_font(args.font)
    directory = args.data / args.scene
    config = yaml.safe_load((directory / 'config.yaml').read_text())
    def matching(row):
        return int(row['swing']) == args.swing and row['method'] == args.method
    curves = sorted(filter(matching, rows(directory / 'trajectories.csv')), key=lambda r: float(r['t']))
    controls = sorted(filter(matching, rows(directory / 'spline_controls.csv')), key=lambda r: int(r['index']))
    knots = sorted(filter(matching, rows(directory / 'knots.csv')), key=lambda r: int(r['index']))
    exported = list(filter(matching, rows(directory / 'hulls.csv')))
    if not curves or not exported:
        raise SystemExit('没有指定轨迹的数据，请先运行 minvo_scene 导出')
    control_points = np.c_[column(controls, 'x'), column(controls, 'z_minus_radius')]
    knot_values = column(knots, 'knot')
    degree = len(knot_values) - len(control_points) - 1
    spans = {}
    metrics = []
    for index in sorted({int(r['span']) for r in exported}):
        group = [r for r in exported if int(r['span']) == index]
        start, end = float(group[0]['start']), float(group[0]['end'])
        # Native local support: only p+1 original B-spline controls are active on a knot span.
        # Bezier/MINVO vertices come directly from the C++ polynomial hull exporter.
        k = int(np.searchsorted(knot_values, (start + end) / 2, side='right') - 1)
        hulls = {'B-spline': control_points[k-degree:k+1]}
        for basis in ['Bezier', 'MINVO']:
            vertices = sorted((r for r in group if r['basis'] == basis), key=lambda r: int(r['vertex']))
            hulls[basis] = np.c_[column(vertices, 'x'), column(vertices, 'z')]
        samples = [r for r in curves if start-1e-10 <= float(r['t']) <= end+1e-10]
        if len(samples) < 3:
            raise SystemExit('分段采样不足，请先提高生产导出采样密度')
        curve_points = np.c_[column(samples, 'x'), column(samples, 'z')]
        # This checks only the exported samples, not a new continuous collision certificate.
        for basis, _ in BASES:
            boundary = np.asarray(polygon(hulls[basis]))
            if len(boundary) >= 3:
                edges = np.roll(boundary, -1, axis=0) - boundary
                rel = curve_points[:, None, :] - boundary[None, :, :]
                cross = edges[None, :, 0]*rel[:, :, 1] - edges[None, :, 1]*rel[:, :, 0]
                assert np.min(cross) >= -1e-9, (args.scene, index, basis, 'sample outside hull')
        areas = {b: area(hulls[b]) for b, _ in BASES}
        row = dict(scene=args.scene, swing=args.swing, method=args.method, span=index,
                   start=start, end=end, bspline_area_m2=areas['B-spline'],
                   bezier_area_m2=areas['Bezier'], minvo_area_m2=areas['MINVO'],
                   sampled_curve_hull_area_m2=area(curve_points),
                   minvo_over_bezier=areas['MINVO']/areas['Bezier'] if areas['Bezier'] > 1e-14 else float('nan'),
                   minvo_over_bspline=areas['MINVO']/areas['B-spline'] if areas['B-spline'] > 1e-14 else float('nan'))
        metrics.append(row)
        spans[index] = (hulls, samples, row)
    candidates = [r for r in metrics if r['bezier_area_m2'] > 1e-12]
    if not candidates:
        raise SystemExit('所有包络在 XZ 平面退化，无法进行面积比较')
    selected = args.span if args.span is not None else min(candidates, key=lambda r: r['minvo_over_bezier'])['span']
    if selected not in spans:
        raise SystemExit(f'分段不存在，可选：{list(spans)}')
    hulls, samples, selected_metrics = spans[selected]
    stem = f'{args.scene}_swing{args.swing}_{args.method}_local_envelopes'
    args.result.mkdir(parents=True, exist_ok=True)
    with (args.data / f'{stem}.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    bezier_sum = sum(r['bezier_area_m2'] for r in metrics)
    minvo_sum = sum(r['minvo_area_m2'] for r in metrics)
    colors = dict(BASES)
    def plot_envelopes(ax, indexes, show_vertices):
        for basis in ['Bezier', 'MINVO']:
            for j, index in enumerate(indexes):
                vertices = spans[index][0][basis]
                boundary = polygon(vertices)
                ax.add_patch(Polygon(boundary, facecolor=colors[basis],
                                     edgecolor=colors[basis], alpha=.22 if basis == 'Bezier' else .35,
                                     linewidth=1.6, label=basis+' 包络' if j == 0 else None,
                                     zorder=2 if basis == 'Bezier' else 3))
                if show_vertices:
                    ax.scatter(vertices[:, 0], vertices[:, 1], c=colors[basis], s=28,
                               edgecolors='white', linewidths=.4, zorder=5)
        ax.grid(alpha=.2)
        ax.set(xlabel='前进方向 x (m)', ylabel='足端高度 z (m)')
        ax.legend(loc='best', fontsize=10)

    fig, ax = plt.subplots(figsize=(10, 5.5), layout='constrained')
    for x0, x1, y0, y1, height in config['obstacles']:
        if y0 <= float(curves[0]['y']) <= y1:
            ax.fill_between([x0, x1], 0, height, color='#a8a8a8', alpha=.35)
    plot_envelopes(ax, list(spans), False)
    ax.plot(column(curves,'x'), column(curves,'z'), color='#253442', linewidth=1.4,
            label='同一条 MINVO 优化曲线', zorder=6)
    for index in spans:
        row = spans[index][2]
        mid = min(curves, key=lambda r: abs(float(r['t'])-(row['start']+row['end'])/2))
        ax.annotate(str(index), (float(mid['x']),float(mid['z'])),
                    xytext=(0,9), textcoords='offset points', fontsize=8, ha='center')
    ax.plot(column(curves,'x')[[0,-1]],column(curves,'z')[[0,-1]],'o',
            color='#253442', markerfacecolor='white', zorder=7)
    ax.legend(loc='lower center', fontsize=10)
    ax.set_aspect('equal', adjustable='box')
    ax.margins(x=.06,y=.20)
    ax.set_title('单障碍：完整轨迹在 XZ 平面的分段包络叠加（等比例坐标）\n'
                 f'同一曲线、同一原始分段；面积总和：Bezier {bezier_sum*1e6:.1f} mm²，'
                 f'MINVO {minvo_sum*1e6:.1f} mm²')
    for extension in ['png','svg']:
        fig.savefig(args.result / f'single_envelopes_full.{extension}')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9,5.5), layout='constrained')
    # Rigidly rotate the XZ projection onto its chord, then magnify the normal
    # coordinate in display only. Rotation preserves the reported physical area.
    curve_points = np.c_[column(samples, 'x'), column(samples, 'z')]
    origin = curve_points[0]
    tangent = curve_points[-1] - origin
    tangent /= np.linalg.norm(tangent)
    rotation = np.array([tangent, [-tangent[1], tangent[0]]]).T
    def local_coordinates(points):
        return (points - origin) @ rotation * 1000  # mm
    for basis in ['Bezier', 'MINVO']:
        vertices = local_coordinates(hulls[basis])
        ax.add_patch(Polygon(polygon(vertices), facecolor=colors[basis], edgecolor=colors[basis],
                             alpha=.30 if basis == 'Bezier' else .45, linewidth=2,
                             label=basis+' 包络', zorder=2 if basis == 'Bezier' else 3))
        ax.scatter(vertices[:, 0], vertices[:, 1], c=colors[basis], s=40,
                   edgecolors='white', linewidths=.6, zorder=5)
    rotated_curve = local_coordinates(curve_points)
    ax.plot(rotated_curve[:, 0], rotated_curve[:, 1], color='#253442', linewidth=1.6,
            label='同一曲线段', zorder=6)
    ax.set(xlabel='沿曲线弦方向的距离 (mm)', ylabel='垂直弦方向的偏差 (mm)')
    ax.grid(alpha=.2)
    local = local_coordinates(np.vstack([hulls['Bezier'],hulls['MINVO']]))
    lo, hi = local.min(axis=0), local.max(axis=0)
    pad = np.maximum((hi-lo)*.12,1e-5)
    ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]),ylim=(lo[1]-pad[1],hi[1]+pad[1]))
    ratio = selected_metrics['minvo_over_bezier']
    ax.set_title(f'单障碍：第 {selected} 段包络放大（XZ 平面旋转至弦方向，法向放大显示）\n'
                 f't = {selected_metrics["start"]:.3f}–{selected_metrics["end"]:.3f} s；'
                 f'Bezier {selected_metrics["bezier_area_m2"]*1e6:.3f} mm²，'
                 f'MINVO {selected_metrics["minvo_area_m2"]*1e6:.3f} mm²；减少 {(1-ratio)*100:.1f}%')
    ax.legend(loc='best')
    for extension in ['png','svg']:
        fig.savefig(args.result / f'single_envelopes_best_span.{extension}')
    plt.close(fig)
    print(f'Full span area sum MINVO/Bezier={minvo_sum/bezier_sum:.6f}; best span={selected}, ratio={ratio:.6f}')
    print('输出两张单图（PNG/SVG）；全部原始分段面积指标保留在 CSV。')


if __name__ == '__main__':
    main()
