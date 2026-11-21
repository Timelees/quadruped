#!/usr/bin/env python3
"""Audit raw WBC matrices and project a real acceleration slice. Use system SciPy.
Run with PYTHONNOUSERSITE=1 /usr/bin/python3; no controller parameters are tuned here.
"""
import argparse
import csv
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull


def load(path):return [json.loads(line) for line in path.read_text().splitlines()]
def write_csv(path,rows):
    assert rows
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def arr(s,key):return np.array(s[key],dtype=float)
def vec(s,key):return arr(s,key).reshape(-1)


def physical(s,leg):
    q=vec(s,'q');base=vec(s,'baseline');n=len(q);nm=arr(s,'J').shape[1]
    E=arr(s,'E');e=vec(s,'e');D=arr(s,'D');d=vec(s,'d')
    if not s['actuator_valid']:return None
    # Exactly the actuator, body/support and swing reserves used by safety WBC.
    E=np.vstack([E,np.eye(n)[:6]]);e=np.r_[e,base[:6]]
    idx=np.arange(nm,nm+12);force=np.eye(n)[idx]
    torque=np.eye(n)[-12:]
    D=np.vstack([D,torque,-torque,force,-force])
    d=np.r_[d,vec(s,'torque_upper'),-vec(s,'torque_lower'),base[idx]+s['force_delta'],-base[idx]+s['force_delta']]
    J=arr(s,'J');G=np.zeros((12,n));G[:,:nm]=J;drift=vec(s,'drift')
    swing=[i for i,bit in enumerate([8,4,2,1]) if not s['mode']&bit]
    ids=np.concatenate([np.arange(3*i,3*i+3) for i in swing]) if swing else np.array([],dtype=int)
    if len(ids):
        D=np.vstack([D,G[ids],-G[ids]]);d=np.r_[d,G[ids]@base+s['accel_delta'],-G[ids]@base+s['accel_delta']]
    return E,e,D,d,G[3*leg:3*leg+3],drift[3*leg:3*leg+3]


def lp(c,E,e,D,d):
    return linprog(c,A_ub=D,b_ub=d,A_eq=E,b_eq=e,bounds=[(None,None)]*len(c),method='highs',
                   options={'dual_feasibility_tolerance':1e-8,'primal_feasibility_tolerance':1e-8})


def polygon(E,e,D,d,G,drift):
    # Project into (ux,uz) at the ACTUAL uy; all returned vertices carry a full
    # feasible WBC witness. Adapt edges until every support gap is <=1e-6.
    points=[];witness=[]
    def support(a):
        result=lp(-a@G[[0,2]],E,e,D,d)
        if not result.success:return None
        point=G[[0,2]]@result.x+drift[[0,2]]
        return point,result.x
    for theta in np.linspace(0,2*np.pi,16,endpoint=False):
        r=support(np.array([np.cos(theta),np.sin(theta)]))
        if r is None:return dict(points=[],empty=True)
        points.append(r[0]);witness.append(r[1])
    converged=False
    for _ in range(20):
        p=np.array(points);hull=ConvexHull(p);added=0
        for eq in hull.equations:
            a=eq[:2];r=support(a)
            if a@r[0]+eq[2]>1e-6:
                points.append(r[0]);witness.append(r[1]);added+=1
        if not added:converged=True;break
    hull=ConvexHull(points);ids=hull.vertices
    return dict(points=np.array(points)[ids].tolist(),witnesses=np.array(witness)[ids].tolist(),
                area=float(hull.volume),empty=False,support_converged=converged,tolerance=1e-6)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();root=a.directory
    task=(root/'gazebo_pilot.task.info').read_text()
    def setting(name):
        values=re.findall(r'^\s*'+re.escape(name)+r'\s+([+\-\d.eE]+)\b',task,re.M)
        assert len(values)==1,(name,values)
        return float(values[0])
    gain1=setting('safetyFilterHocbfGain1');gain2=setting('safetyFilterHocbfGain2')
    disturbance=setting('safetyFilterDisturbanceBound');sample_rate=setting('safetyFilterHocbfSampleRate');period=setting('safetyFilterMaxPeriod')
    samples=load(root/'wbc.jsonl');swings=load(root/'wbc.jsonl.swings.jsonl')
    assert samples and swings
    durations=np.array([s['end']-s['start'] for s in swings]);trot=durations[np.abs(durations-.4)<1e-4]
    assert len(trot)>0,'No emitted 0.4 s MPC swings'
    schedule=dict(n=len(durations),trot_n=len(trot),median_s=float(np.median(trot)),
        minimum_s=float(trot.min()),maximum_s=float(trot.max()),bins_ms=(np.arange(1,9)*np.median(trot)/8*1000).tolist())
    (root/'runtime_schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
    rows=[];plane_rows=[]
    for s in samples:
        q=vec(s,'q');J=arr(s,'J');u=(J@q[:J.shape[1]]+vec(s,'drift')).reshape(4,3)
        for leg in range(4):
            plane=arr(s,'planes_'+str(leg))
            if not plane.size:continue
            for j,r in enumerate(plane):
                normal=r[:3];h,hd=r[5:7];hold=[]
                for k in range(3):
                    t=period*k/2;tt=period**2 if k==2 else 0;gain=1+(gain1+gain2)*t+.5*gain1*gain2*tt
                    hold.append((gain1+gain2+gain1*gain2*t)*hd+gain1*gain2*h-gain*disturbance*np.linalg.norm(normal)-sample_rate*period-gain*normal@u[leg])
                plane_rows.append(dict(time=s['time'],leg=leg,plane=j,nx=r[0],ny=r[1],nz=r[2],offset=r[3],
                    h_raw=r[4],h_robust=h,psi1_robust=r[7],R_zero_hold=r[8]-normal@u[leg],R_hold=min(hold),
                    status=s['status_'+str(leg)],actuator_valid=s['actuator_valid']))
            relevant=plane_rows[-len(plane):]
            P=physical(s,leg)
            r_input=np.nan;r_eq=np.nan
            if P:
                E,e,D,d,G,drift=P;r_input=float(np.max(D@q-d));r_eq=float(np.max(np.abs(E@q-e)))
            rows.append(dict(time=s['time'],leg=leg,status=s['status_'+str(leg)],actuator_valid=s['actuator_valid'],
                h_min=min(r['h_robust'] for r in relevant),psi1_min=min(r['psi1_robust'] for r in relevant),
                R_min=min(r['R_hold'] for r in relevant),input_witness_ineq=r_input,input_witness_eq=r_eq,
                ux=u[leg,0],uy=u[leg,1],uz=u[leg,2],nominal_x=arr(s,'nominal')[0,leg],nominal_z=arr(s,'nominal')[2,leg],
                torque_usage=float(np.max(np.abs(q[-12:]))/12)))
    assert rows,'No HOCBF planes collected'
    write_csv(root/'hocbf_planes.csv',plane_rows);write_csv(root/'control_audit.csv',rows)
    # One full swing with most model-valid HOCBF evidence; tie-break by proximity
    # to obstacles, never by whether all residuals are positive.
    candidates=[]
    for sw in swings:
        selected=[r for r in rows if r['leg']==sw['leg'] and sw['start']<=r['time']<=sw['end']]
        if selected:
            score=sum(s['actuator_valid'] and bool(s['H']) and s['status_'+str(sw['leg'])] in [1,13,15,16] for s in samples if sw['start']<=s['time']<=sw['end'])
            candidates.append((score,sw,selected))
    candidates=[v for v in candidates if v[0]>0]
    assert candidates,'No swing with actual model-valid hard safety rows'
    _,sw,selected=max(candidates,key=lambda v:(v[0],-abs(v[1]['controls'][0][0]+1.2)))
    (root/'selected_swing.json').write_text(json.dumps(sw,indent=2)+'\n')
    write_csv(root/'selected_control.csv',selected)
    # Preserve all potential snapshot candidates and the deterministic rule.
    snapshot_candidates=[]
    for s in samples:
        leg=sw['leg']
        if sw['start']<=s['time']<=sw['end'] and s['actuator_valid'] and len(s['H']) and s['status_'+str(leg)] in [1,13,15,16]:
            snapshot_candidates.append(s)
    assert snapshot_candidates,'No physical model + safety constraint snapshot in selected swing'
    s=max(snapshot_candidates,key=lambda s:float(s['safety_projection']))
    E,e,D,d,G,drift=physical(s,sw['leg']);q=vec(s,'q');actual=G@q+drift
    # Fix uy, project ux/uz while allowing all admissible joint/force changes.
    E=np.vstack([E,G[1]]);e=np.r_[e,actual[1]-drift[1]]
    model=polygon(E,e,D,d,G,drift)
    H=arr(s,'H');hb=vec(s,'h');joint=polygon(E,e,np.vstack([D,H]),np.r_[d,hb],G,drift)
    snap=dict(time=s['time'],leg=sw['leg'],actual=actual.tolist(),nominal=arr(s,'nominal')[:,sw['leg']].tolist(),
        physical=model,joint=joint,selection='Full swing: most model-valid hard-safety samples, then nearest first obstacle; snapshot: first actual safety projection, otherwise first constrained sample.',
        safety_projection=s['safety_projection'],E=E.tolist(),e=e.tolist(),D=D.tolist(),d=d.tolist(),H=H.tolist(),h=hb.tolist(),
        G=G.tolist(),drift=drift.tolist(),q=q.tolist())
    (root/'input_slice.json').write_text(json.dumps(snap,indent=2)+'\n')
    # True per-tick directional support bounds, not a constant ±80 acceleration box.
    bounds=[]
    for s in snapshot_candidates:
        E,e,D,d,G,drift=physical(s,sw['leg']);record=dict(time=s['time'])
        for axis in [0,2]:
            for sign in [-1,1]:
                res=lp(sign*G[axis],E,e,D,d)
                record[f'{"xz"[axis//2]}_{"max" if sign<0 else "min"}']=float(G[axis]@res.x+drift[axis]) if res.success else None
        bounds.append(record)
    write_csv(root/'input_bounds.csv',bounds)
    (root/'analysis_summary.json').write_text(json.dumps(dict(samples=len(samples),swings=len(swings),
        plane_samples=len(plane_rows),active_foot_samples=len(rows),actuator_valid=sum(r['actuator_valid'] for r in rows),
        h_negative=int(sum(r['h_min']<-1e-8 for r in rows)),psi_negative=int(sum(r['psi1_min']<-1e-8 for r in rows)),
        R_negative=int(sum(r['R_min']<-1e-8 for r in rows)),
        input_witness_violations=sum(r['input_witness_ineq']>1e-5 or r['input_witness_eq']>1e-5 for r in rows),
        selected_swing_start=sw['start'],selected_swing_end=sw['end']),indent=2)+'\n')
    print((root/'analysis_summary.json').read_text())

if __name__=='__main__':main()
