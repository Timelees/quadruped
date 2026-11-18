#!/usr/bin/env python3
"""Summarize retained trials without discarding failures or treating samples as trials."""
import argparse
import csv
import json
import re
from pathlib import Path
import numpy as np


def load(path):
    return np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True, encoding=None))


def analyze(stem):
    info = json.loads(stem.with_suffix('.json').read_text())
    row = dict(trial=stem.name, scenario=info['scenario'], result=info['result'],
               speed=info['speed_normalized'], max_roll=info['max_abs_roll'],
               max_pitch=info['max_abs_pitch'], duration=info['final'][0],
               final_x=info['final'][1], final_y=info['final'][2])
    logpath = stem.with_suffix('.launch.log')
    row['launch_log_available'] = logpath.exists()
    log = logpath.read_text(errors='replace') if logpath.exists() else ''
    plans = re.findall(r'Swing planning ms=([\d.eE+-]+) corridor_cells=(\d+) corridor_certified=(\d) corridor_max_violation=([^\s]+)', log)
    times = np.array([float(p[0]) for p in plans])
    row.update(plans=len(plans), certified=sum(p[2]=='1' for p in plans),
               fallbacks=log.count('Swing terrain-QP fallback'),
               planning_p50_ms=float(np.median(times)) if len(times) else None,
               planning_p95_ms=float(np.percentile(times,95)) if len(times) else None)
    wpath = stem.with_name(stem.name+'_wbc.csv')
    events = []
    if wpath.exists() and wpath.stat().st_size > 100:
        w = load(wpath)
        row['invalid_wbc'] = int(np.count_nonzero(w['valid']==0))
        row['wbc_nwsr_max'] = int(np.max(w['nwsr']))
        row['wbc_residual_max'] = float(np.max(w['constraint_violation']))
        # Drop startup only for tracking/impact summaries; all solver failures remain counted.
        t = w['ros_time'] if 'ros_time' in w.dtype.names else w['sim_time']
        mask = t >= t[0]+5
        errors=[]; peaks=[]
        for leg in ['LF','RF','LH','RH']:
            e = np.sqrt(sum((w[f'{leg}_p_{a}']-w[f'{leg}_pd_{a}'])**2 for a in 'xyz'))
            errors.extend(e[mask].tolist())
            if 'sensor_'+leg not in w.dtype.names: continue
            force=w['sensor_'+leg]; peaks.extend(force[mask].tolist())
            # Contact onset is a measured force crossing, not a planned mode switch.
            for i in np.flatnonzero((force[1:]>=5)&(force[:-1]<5))+1:
                if not mask[i]: continue
                before=np.flatnonzero((t>=t[i]-.035)&(t<t[i]))
                if not len(before):continue
                events.append(dict(trial=stem.name,leg=leg,ros_time=float(t[i]),
                    downward_speed=max(0.,-float(np.min(w[leg+'_v_z'][before]))),
                    force_peak=float(np.max(force[(t>=t[i])&(t<=t[i]+.04)]))))
        row['mpc_tracking_rms_mm']=float(np.sqrt(np.mean(np.square(errors)))*1000) if errors else None
        row['sensor_force_max_N']=float(max(peaks)) if peaks else None
        row['contact_downward_p95_mps']=float(np.percentile([e['downward_speed'] for e in events],95)) if events else None
    return row,events


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--final-trials',nargs='*',default=[])
    a=p.parse_args(); rows=[];events=[]
    for path in sorted(a.directory.glob('*.json')):
        info=json.loads(path.read_text())
        if 'result' not in info or not path.with_suffix('.csv').exists():continue
        row,ev=analyze(path.with_suffix(''));rows.append(row);events+=ev
    a.output.mkdir(parents=True,exist_ok=True)
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with (a.output/'trial_metrics.csv').open('w') as f:
        w=csv.DictWriter(f,keys);w.writeheader();w.writerows(rows)
    if events:
        with (a.output/'contact_events.csv').open('w') as f:
            w=csv.DictWriter(f,list(events[0]));w.writeheader();w.writerows(events)
    selected=[r for r in rows if r['trial'] in a.final_trials]
    if set(a.final_trials)-{r['trial'] for r in selected}:
        raise RuntimeError('A selected final trial is missing; no success-rate report created')
    summary={'final_trials':selected,'all_trials':rows,
             'definitions':{'success':'All feet inside destination, roll <0.20, pitch <0.25 for 3 sim seconds',
             'tracking':'Kinematic foot versus MPC desired foot, not versus the MINVO reference',
             'contact':'Sensor force crosses 5 N; downward speed in previous 35 ms; force peak in next 40 ms',
             'planning':'All attempted horizon swings including fallback; timing includes workspace queries'}}
    (a.output/'stability_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for r in selected:print(r)

if __name__=='__main__':main()
