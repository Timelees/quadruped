#!/usr/bin/env python3
"""Read-only follow-up audit; write separate diagnostics, never revise source data.

Use PYTHONNOUSERSITE=1 /usr/bin/python3 for the installed SciPy runtime.
Dense sphere/box checks detect collisions, but are not continuous certificates.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline

from analyze_extended import lp


def records(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def jsonlines(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def curve(value):
    return BSpline(value['knots'], value['controls'], 5, extrapolate=False)


def clearance(points, height, dx):
    # Production references describe the foot contact point; corridor construction
    # adds footRadius in z to obtain the spherical foot centre. Only the test bar
    # is considered here, without ground, robot links or uncertainty inflation.
    radius = .019
    centres = np.asarray(points) + [0., 0., radius]
    delta = np.abs(centres - [dx, 0., height / 2]) - [.015, .5, height / 2]
    return (np.linalg.norm(np.maximum(delta, 0), axis=1)
            + np.minimum(np.max(delta, axis=1), 0) - radius)


def quantiles(rows):
    values = np.array([float(row['total_ms']) for row in rows])
    return dict(n=len(rows), p50_ms=float(np.quantile(values, .5)),
                p95_ms=float(np.quantile(values, .95)),
                p99_ms=float(np.quantile(values, .99)), maximum_ms=float(values.max()),
                above_5ms=int(sum(values > 5)), above_60ms=int(sum(values > 60)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, output = args.directory, args.output
    if output.exists():
        raise FileExistsError('Use a fresh output directory; source results must be preserved')
    trials = records(root / 'sweep/trials.csv')
    accepted = jsonlines(root / 'sweep/accepted.jsonl')
    assert len(trials) == 6400
    assert len(accepted) == sum(int(row['instantiated']) for row in trials)
    lookup = {(round(float(r['height_m']), 6), round(float(r['remaining_s']), 6),
               int(r['trial'])): r for r in trials}
    sources = {row['source']: row['spline'] for row in jsonlines(root / 'sweep/hulls.jsonl')}
    old_curves = {key: curve(value) for key, value in sources.items()}
    collision_rows = []
    threshold = -1e-6
    for index, sample in enumerate(accepted):
        height, remaining, trial = sample['height'], sample['remaining'], sample['trial']
        row = lookup[(round(height, 6), round(remaining, 6), trial)]
        dx = float(row['obstacle_dx'])
        activation = .4 - remaining
        future = np.linspace(activation, .4, 1601)
        certified = np.linspace(max(activation, .1), .3, 1001)
        new = curve(sample['new'])
        old = old_curves[sample['source']]
        old_future = clearance(old(future), height, dx)
        new_future = clearance(new(future), height, dx)
        old_certified = clearance(old(certified), height, dx)
        new_certified = clearance(new(certified), height, dx)
        collision_ids = np.flatnonzero(new_future < threshold)
        first = float(future[collision_ids[0]]) if len(collision_ids) else None
        collision_rows.append(dict(index=index, height_m=height, remaining_s=remaining,
            trial=trial, source=sample['source'], obstacle_dx=dx,
            old_future_collision=bool(np.min(old_future) < threshold),
            new_future_collision=bool(len(collision_ids)),
            old_certified_collision=bool(np.min(old_certified) < threshold),
            new_certified_collision=bool(np.min(new_certified) < threshold),
            new_start_collision=bool(new_future[0] < threshold),
            new_terminal_collision=bool(new_future[-1] < threshold),
            first_collision_s=first, old_min_clearance_m=float(old_future.min()),
            new_min_clearance_m=float(new_future.min()),
            new_start_clearance_m=float(new_future[0]),
            new_terminal_clearance_m=float(new_future[-1])))
    def collision_summary(rows):
        result = dict(accepted=len(rows))
        for key in ['old_future_collision', 'new_future_collision',
                    'old_certified_collision', 'new_certified_collision',
                    'new_start_collision', 'new_terminal_collision']:
            result[key] = sum(row[key] for row in rows)
        result['old_collision_to_new_no_detected_collision'] = sum(
            r['old_future_collision'] and not r['new_future_collision'] for r in rows)
        result['new_collision_with_clear_start_and_terminal'] = sum(
            r['new_future_collision'] and not r['new_start_collision']
            and not r['new_terminal_collision'] for r in rows)
        result['first_collision_before_certified_entry'] = sum(
            r['first_collision_s'] is not None
            and r['first_collision_s'] < max(.4-r['remaining_s'], .1)-1e-8 for r in rows)
        result['first_collision_after_certified_exit'] = sum(
            r['first_collision_s'] is not None and r['first_collision_s'] > .3+1e-8 for r in rows)
        return result
    groups = {'all': trials,
        'instantiated': [r for r in trials if int(r['instantiated'])],
        'candidate_passed': [r for r in trials if int(r['candidate_certified'])],
        'qp_failed': [r for r in trials if 'QP failed' in r['reason']],
        'corridor_failed': [r for r in trials if 'corridor' in r['reason']]}
    assert all(groups.values()), dict(Counter(r['reason'] for r in trials))
    metrics = records(root / 'hull_metrics.csv')
    hull_summary = {}
    for kind in sorted({r['kind'] for r in metrics}):
        rows = [r for r in metrics if r['kind'] == kind]
        b = np.array([float(r['bernstein_area']) for r in rows])
        m = np.array([float(r['minvo_area']) for r in rows])
        hull_summary[kind] = dict(n=len(rows), improved=int(sum(m < b)),
            median_reduction=float(np.median([float(r['reduction']) for r in rows])),
            segment_area_sum_reduction=float(1-m.sum()/b.sum()))
    audit = records(root / 'control_audit.csv')
    statuses = {}
    for status in sorted({int(r['status']) for r in audit}):
        rows = [r for r in audit if int(r['status']) == status]
        statuses[str(status)] = dict(n=len(rows), **{
            key+'_negative': sum(float(r[key]) < -1e-8 for r in rows)
            for key in ['h_min', 'psi1_min', 'R_min']})
    snap = json.loads((root / 'input_slice.json').read_text())
    # The stored final equality fixes uy for the displayed 2D slice. Remove it
    # before independently checking each full three-dimensional input target.
    E = np.asarray(snap['E'])[:-1]; e = np.asarray(snap['e'])[:-1]
    G = np.asarray(snap['G']); drift = np.asarray(snap['drift'])
    D = np.asarray(snap['D']); d = np.asarray(snap['d'])
    H = np.asarray(snap['H']); h = np.asarray(snap['h'])
    assert np.allclose(np.asarray(snap['E'])[-1], G[1])
    membership = {}
    for name in ['nominal', 'actual']:
        eq = np.vstack([E, G]); rhs = np.r_[e, np.asarray(snap[name])-drift]
        membership[name] = {}
        for label, matrix, bound in [('physical', D, d), ('joint', np.vstack([D,H]), np.r_[d,h])]:
            result = lp(np.zeros(G.shape[1]), eq, rhs, matrix, bound)
            membership[name][label] = dict(feasible=bool(result.success), status=int(result.status))
    replans = json.loads((root / 'gazebo_pilot.replans.json').read_text())
    result = dict(protocol=dict(collision_threshold_m=threshold, future_samples=1601,
        certified_samples=1001, certified_interval_s=[.1,.3], sphere_radius_m=.019,
        sphere_centre_z_offset_m=.019,
        scope='Dense obstacle-only diagnostic, not a continuous safety certificate or physical trial'),
        collision=collision_summary(collision_rows),
        collision_by_height={str(height): collision_summary([r for r in collision_rows
            if abs(r['height_m']-height/100)<1e-8]) for height in range(2,17,2)},
        latency_by_path={key: quantiles(rows) for key, rows in groups.items()},
        reasons=dict(Counter(r['reason'] for r in trials)),
        sampled_unique_sources=len({int(r['source']) for r in trials}),
        hull_population=hull_summary, hocbf_by_status=statuses,
        input_full3d_membership=membership,
        gazebo=dict(decisions=len(replans['decisions']), applied=len(replans['applied']),
            reasons=dict(Counter(r['reason'] for r in replans['decisions']))))
    assert result['collision']['new_certified_collision'] == 0
    output.mkdir(parents=True)
    with (output / 'accepted_collision_audit.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(collision_rows[0]))
        writer.writeheader(); writer.writerows(collision_rows)
    inputs = ['sweep/trials.csv', 'sweep/accepted.jsonl', 'sweep/hulls.jsonl',
              'hull_metrics.csv', 'control_audit.csv', 'input_slice.json', 'gazebo_pilot.replans.json']
    result['source_sha256'] = {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in inputs}
    (output / 'audit_summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: result[key] for key in ['collision', 'latency_by_path',
        'hull_population', 'input_full3d_membership']}, indent=2))


if __name__ == '__main__':
    main()
