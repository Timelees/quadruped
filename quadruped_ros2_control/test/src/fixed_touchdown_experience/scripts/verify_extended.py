#!/usr/bin/env python3
"""Independent source-data checks for the extended figure set."""
import argparse
import csv
import json
import math
import re
from pathlib import Path
import numpy as np
from plot_figures import evaluate


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();root=a.directory
    trials=list(csv.DictReader((root/'sweep/trials.csv').open()));assert len(trials)==6400
    for height in range(2,17,2):
        for b in range(1,9):
            cell=[r for r in trials if abs(float(r['height_m'])-.01*height)<1e-8 and abs(float(r['remaining_s'])-.05*b)<1e-8]
            assert len(cell)==100 and set(int(r['trial']) for r in cell)==set(range(100))
    for r in trials:
        assert float(r['dp_norm'])<=.001+1e-12 and float(r['dv_norm'])<=.01+1e-12 and float(r['da_norm'])<=.5+1e-12
        assert int(r['instantiated'])<=int(r['candidate_certified'])
        assert int(r['terminal_preserved'])==int(r['instantiated'])
    sources={};segments=[json.loads(s) for name in ['hulls.jsonl','hulls_instantiated.jsonl'] for s in (root/'sweep'/name).read_text().splitlines()]
    accepted=[json.loads(s) for s in (root/'sweep/accepted.jsonl').read_text().splitlines()]
    code=next(p for p in root.resolve().parents if (p/'src/quadruped_ros2_control/controllers').is_dir())
    source=(code/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/src/interface/constraint/PolynomialHull.cpp').read_text()
    coeff=source.split('value <<',1)[1].split(';',1)[0]
    basis=np.array([float(s) for s in re.findall(r'[-+]?\d+\.\d+',coeff)]).reshape(6,6)
    max_basis_error=0;min_weight=1
    for s in segments:
        sources[s['source']]=s['spline'];u=np.linspace(0,1,301);tt=s['start']+u*(s['end']-s['start'])
        expected=evaluate(accepted[s['parent_index']]['new'] if 'parent_index' in s else s['spline'],tt)
        wb=np.array([math.comb(5,i)*u**i*(1-u)**(5-i) for i in range(6)]).T
        powers=np.array([(2*u-1)**i for i in range(5,-1,-1)]).T;wm=powers@basis.T
        assert np.max(np.abs(wm.sum(axis=1)-1))<1e-11
        min_weight=min(min_weight,float(wm.min()));assert min_weight>=-1e-10
        for weights,key in [(wb,'bernstein'),(wm,'minvo')]:
            max_basis_error=max(max_basis_error,float(np.max(np.abs(weights@np.array(s[key])-expected))))
    assert max_basis_error<1e-9
    terminal=np.zeros(3);accepted=[json.loads(s) for s in (root/'sweep/accepted.jsonl').read_text().splitlines()]
    assert len(accepted)==sum(int(r['instantiated']) for r in trials)
    for s in accepted:
        for k in range(3):terminal[k]=max(terminal[k],float(np.max(np.abs(evaluate(s['new'],[.4],k)-evaluate(sources[s['source']],[.4],k)))))
    assert max(terminal)<1e-5
    snap=json.loads((root/'input_slice.json').read_text());E=np.array(snap['E']);e=np.array(snap['e']);D=np.array(snap['D']);d=np.array(snap['d']);H=np.array(snap['H']);h=np.array(snap['h']);G=np.array(snap['G']);drift=np.array(snap['drift'])
    max_witness=0
    for key in ['physical','joint']:
        if snap[key]['empty']:continue
        assert snap[key]['support_converged']
        for point,q in zip(snap[key]['points'],snap[key]['witnesses']):
            q=np.array(q);error=max(float(np.max(np.abs(E@q-e))),float(np.max(D@q-d)),float(np.max(np.abs((G@q+drift)[[0,2]]-point))))
            if key=='joint':error=max(error,float(np.max(H@q-h)))
            max_witness=max(max_witness,error)
    assert max_witness<1e-5
    scene=json.loads((root/'scene.json').read_text())
    assert [r['experiment_top'] for r in scene['obstacles']]==[.02*k for k in range(1,9)]
    result=dict(n=6400,cells=64,trials_per_cell=100,independent_random_seeds=100,
        paired_across_cells=True,accepted=len(accepted),terminal_errors_pva=terminal.tolist(),
        polynomial_basis_max_error=max_basis_error,minvo_min_sampled_weight=min_weight,
        hull_segments=len(segments),input_polygon_witness_max_error=max_witness,
        physical_trials_per_cell=0,workspace_checked_in_replay=False,wbc_checked_in_replay=False)
    (root/'independent_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
