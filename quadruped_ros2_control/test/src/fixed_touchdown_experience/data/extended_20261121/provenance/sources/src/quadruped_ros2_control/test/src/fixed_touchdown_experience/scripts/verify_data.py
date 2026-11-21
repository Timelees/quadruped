#!/usr/bin/env python3
"""Cross-check independently evaluated logged splines against native C++ samples."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from plot_figures import evaluate, clearance


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mechanism',type=Path);a=p.parse_args()
    events=[json.loads(s) for s in (a.mechanism/'mechanism_events.jsonl').read_text().splitlines()]
    assert events,'No admitted mechanism cases'
    rows=list(csv.DictReader((a.mechanism/'mechanism_trace.csv').open()))
    t=np.array([float(r['time_s']) for r in rows]);e=events[0]
    residual={}
    for prefix in ('old','planned','new'):
        for order,quantity in enumerate(('z','vz','az')):
            field=prefix+'_'+quantity
            selected=t>=e['now'] if prefix=='new' else t>=e['reveal'] if prefix=='planned' else np.ones(len(t),dtype=bool)
            actual=evaluate(e[prefix],t[selected],order)[:,2]
            expected=np.array([float(r[field]) for r in rows])[selected]
            err=float(np.max(np.abs(actual-expected)));assert err<1e-7,(field,err);residual[field]=err
    terminal=[];hull_residuals=[]
    for e in events:
        values=[float(np.linalg.norm(evaluate(e['new'],[e['end']],order)-evaluate(e['old'],[e['end']],order))) for order in range(3)]
        assert max(values)<1e-6,values
        terminal.append(values)
        assert e['old_clearance']<0 and e['new_clearance']>0
        if 'certificate' in e:
            certificate=e['certificate'];controls=np.asarray(e['new']['controls'])
            for cell in certificate['cells']:
                hull=np.asarray(cell['mapping'])@controls+[0,0,certificate['foot_radius']]
                planes=np.asarray(cell['planes'])
                hull_error=hull@planes[:,:3].T-planes[:,3]+planes[:,4]+certificate['numerical_margin']
                maximum=float(np.max(hull_error));assert maximum<=1e-8,maximum
                hull_residuals.append(maximum)
    summary=dict(accepted_cases=len(events),cpp_trace_residuals=residual,
                 terminal_max_residuals=np.max(terminal,axis=0).tolist(),
                 minvo_cells_checked=len(hull_residuals),minvo_max_violation=max(hull_residuals,default=None),
                 evidence='independent NumPy evaluation versus native production C++ spline evaluations')
    (a.mechanism/'independent_audit.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))

if __name__=='__main__':main()
