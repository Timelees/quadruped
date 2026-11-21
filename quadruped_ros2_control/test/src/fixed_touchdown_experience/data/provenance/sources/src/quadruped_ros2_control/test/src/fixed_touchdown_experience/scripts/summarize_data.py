#!/usr/bin/env python3
"""Audit all launches without treating planning admission as physical activation."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re


def finite(rows, field):
    return [float(r[field]) for r in rows if field in r and math.isfinite(float(r[field]))]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('data',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();runs=[]
    for path in sorted(a.data.glob('*.json')):
        result=json.loads(path.read_text())
        if not isinstance(result,dict) or 'result' not in result:continue
        manifest=json.loads(path.with_suffix('.manifest.json').read_text())
        log=path.with_suffix('.launch.log').read_text(errors='replace')
        with path.with_name(path.stem+'_wbc.csv').open() as f:rows=list(csv.DictReader(f))
        commits=[json.loads(line.split('SWING_EVIDENCE ',1)[1]) for line in log.splitlines() if 'SWING_EVIDENCE ' in line]
        residuals=[x for leg in ('LF','RF','LH','RH') for x in finite(rows,leg+'_residual_bound_lower')]
        gate_reasons=Counter(re.findall(r'Swing package rejected reason=([^\s]+)',log))
        gate_reasons.update(re.findall(r'Swing activation accepted=0 .*?reason=([^\s]+)',log))
        reveal=json.loads(path.with_suffix('.reveal.json').read_text())
        trigger=next((e for e in reveal if e['event']=='trigger'),None)
        committed=re.findall(r'Swing reveal map committed controller_time=([\d.]+)',log)
        planning_ends=[]
        if trigger is not None:
            for activation,remaining in re.findall(r'Swing replan leg=\d+ activation=(\S+) remaining=(\S+)',log):
                if trigger['controller_time']<=float(activation)<trigger['controller_time']+.4:
                    planning_ends.append(float(activation)+float(remaining))
        runs.append(dict(run=path.stem,result=result['result'],safety=result.get('safety_filter'),
            speed=result['speed_normalized'],controller_sha256=manifest.get(
                'install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so'),
            manifest_sha256=hashlib.sha256(path.with_suffix('.manifest.json').read_bytes()).hexdigest(),
            planning_admissions=(result.get('replanning') or {}).get('nAdmit',0),
            physical_commits=len(commits),applied_updates=result.get('replan_applied',0),
            invalid_wbc_samples=sum(float(r['valid'])!=1 for r in rows),wbc_samples=len(rows),
            max_compute_ms=max(finite(rows,'wbc_compute_ms'),default=None),
            average_residual_lower_bound_samples=len(residuals),
            max_average_residual_lower_bound=max(residuals,default=None),
            average_residual_lower_bound_above_configured_0p5=sum(x>.5 for x in residuals),
            rejection_reasons=dict(gate_reasons),reveal_trigger=trigger,
            controller_map_release_times=[float(x) for x in committed],
            observed_liftoff_plus_duration=None if trigger is None else trigger['liftoff']+trigger['swing_duration_s'],
            planner_end_times_after_trigger=sorted(set(round(t,9) for t in planning_ends)),
            definition='Residual lower bound can falsify the 0.5 m/s² budget; it does not prove a bound. Route pass does not imply online repair.'))
    output=dict(runs=runs,independent_launches=len(runs),
        note='Different binary hashes/protocols are separate batches. No pooled success probability or HOCBF causal claim.')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps([dict(run=r['run'],result=r['result'],commits=r['physical_commits'],
        residual_budget_exceedances=r['average_residual_lower_bound_above_configured_0p5']) for r in runs]))

if __name__=='__main__':main()
