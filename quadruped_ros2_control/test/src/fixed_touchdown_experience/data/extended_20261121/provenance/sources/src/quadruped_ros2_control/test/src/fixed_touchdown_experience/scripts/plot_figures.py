#!/usr/bin/env python3
"""Evidence plots: real Gazebo observations and separately labelled production-algorithm computations."""
import argparse
import csv
import json
import os
from pathlib import Path
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

BLUE, ORANGE, GREEN = '#1f77b4', '#ff7f0e', '#16836f'
LEGS = ('LF','RF','LH','RH')
SOURCE_OUTPUT = None
SKILL = Path(os.environ.get('NATURE_FIGURE_SKILL',str(Path.home()/'.codex/skills/nature-figure')))/'scripts'
sys.path.insert(0, str(SKILL))
from audit_panel_alignment import require_matplotlib_panel_alignment
mpl.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],'font.size':9,'axes.titlesize':11,
    'axes.labelsize':10,'xtick.labelsize':9,'ytick.labelsize':9,'legend.fontsize':8,
    'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','savefig.dpi':600,
    'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,
    'grid.color':'.91','grid.linewidth':.7,'lines.linewidth':1.8})


def rows(path):
    with Path(path).open() as f:return list(csv.DictReader(f))


def evaluate(value, time, derivative=0):
    # Cox–de Boor recurrence, independently evaluated from logged production
    # control points. Avoid mixing incompatible system/user SciPy binaries.
    knots=np.asarray(value['knots'],dtype=float)
    controls=np.asarray(value['controls'],dtype=float);degree=5
    for _ in range(derivative):
        denominator=knots[degree+1:-1]-knots[1:-degree-1]
        controls=np.divide(degree*np.diff(controls,axis=0),denominator[:,None],
                           out=np.zeros((len(controls)-1,3)),where=denominator[:,None]>0)
        knots=knots[1:-1];degree-=1
    t=np.clip(np.atleast_1d(time),knots[0],np.nextafter(knots[-1],knots[0]))
    basis=((t[:,None]>=knots[:-1])&(t[:,None]<knots[1:])).astype(float)
    for order in range(1,degree+1):
        count=len(knots)-order-1
        dl=knots[order:order+count]-knots[:count]
        dr=knots[order+1:order+1+count]-knots[1:count+1]
        left=np.divide(t[:,None]-knots[:count],dl,out=np.zeros((len(t),count)),where=dl>0)
        right=np.divide(knots[order+1:order+1+count]-t[:,None],dr,out=np.zeros((len(t),count)),where=dr>0)
        basis=left*basis[:,:count]+right*basis[:,1:count+1]
    return basis@controls


def clearance(points, height, centre=-1.2, radius=.019):
    p=np.atleast_2d(points)
    q=np.abs(p-np.array([centre,0,height/2]))-np.array([.015,.5,height/2])
    d=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)-radius
    return np.minimum(p[:,2]-radius,d)


def save(fig, target, note, count=1):
    # Dedicated legend strips keep text outside the data area and grid paths.
    for ax in fig.axes:
        handles,labels=ax.get_legend_handles_labels()
        if handles:
            if ax.get_legend() is not None:ax.get_legend().remove()
            ax.legend(handles,labels,loc='lower right',bbox_to_anchor=(1,1.03),ncol=2,
                      fontsize=8,frameon=False,borderaxespad=0)
    fig.canvas.draw()
    if count>1:
        require_matplotlib_panel_alignment(fig,json_out=target.with_suffix('.alignment.json'),
            tolerance_pt=1.5,gutter_tolerance_pt=1.5,require_panel_labels=True,strict=True)
    for ext in ('.pdf','.svg','.png'):fig.savefig(target.with_suffix(ext),dpi=600)
    target.with_suffix('.caption.txt').write_text(note+'\n')
    plt.close(fig)


def panels(n,title):
    fig,axs=plt.subplots(n,1,figsize=(7.2,3.7 if n==1 else 2.3*n+.6),sharex=True)
    if n==1:axs=np.array([axs])
    fig.subplots_adjust(left=.14,right=.98,bottom=.19 if n==1 else .11,top=.77 if n==1 else .86,hspace=.65)
    fig.suptitle(title,fontsize=12,y=.98)
    if n>1:
        for i,ax in enumerate(axs):ax.text(-.105,1.08,chr(97+i),transform=ax.transAxes,weight='bold',va='bottom',fontsize=10)
    return fig,axs


def event_score(e):
    t=np.linspace(max(e['now'],e['start']+.25*(e['end']-e['start'])),e['start']+.75*(e['end']-e['start']),1201)
    old=clearance(evaluate(e['old'],t)+[0,0,.019],e['height']).min()
    new=clearance(evaluate(e['new'],t)+[0,0,.019],e['height']).min()
    return bool(old<0 and new>0),old,new


def trajectory_figures(e,out):
    t=np.linspace(e['start'],e['end'],4001);active=t>=e['now']
    old=evaluate(e['old'],t);new=old.copy();new[active]=evaluate(e['new'],t[active]);
    prefix='Gazebo activation' if e.get('physical_commit') else 'Offline production algorithm'
    duration=e['end']-e['start'];tau=t-e['start'];ta=e['now']-e['start']
    fig,(ax,)=panels(1,'In-swing obstacle discovery and trajectory update')
    fig.text(.98,.035,prefix,ha='right',color='.35',fontsize=8)
    fig.text(.14,.035,f"Update {e.get('reveal',e['now'])-e['start']:.3f} s; instantiate {ta:.3f} s; fixed touchdown {duration:.3f} s",fontsize=8)
    ax.add_patch(Rectangle((-1.8,-.035),1.1,.035,facecolor=BLUE,alpha=.15,label='Ground'))
    ax.add_patch(Rectangle((-1.215,0),.03,e['height'],facecolor=BLUE,alpha=.32,label='Obstacle'))
    ax.plot(old[:,0],old[:,2],'--',color=BLUE,label='Original reference')
    ax.plot(new[:,0],new[:,2],color=ORANGE,label='Activated reference' if e.get('physical_commit') else 'Instantiated reference')
    envelope=active&(t>=e['start']+.25*duration)&(t<=e['start']+.75*duration)
    ax.fill_between(new[envelope,0],new[envelope,2]-.02,new[envelope,2]+.02,
                    color=ORANGE,alpha=.12,label='Planning allowance (±20 mm)')
    trigger=evaluate(e['old'],[e.get('reveal',e['now'])])[0]
    ax.axvline(trigger[0],color=BLUE,ls=':',lw=1.2,label='Update trigger')
    ax.set(xlabel='Foot reference x (m)',ylabel='Foot reference z (m)',ylim=(-.01,max(e['height']+.07,float(new[:,2].max())+.02)))
    ax.set_xlim(min(old[:,0].min(),new[:,0].min())-.02,max(old[:,0].max(),new[:,0].max())+.02)
    ax.invert_xaxis();ax.legend(loc='upper center',ncol=2,bbox_to_anchor=(.5,1.0))
    save(fig,out/'fig1_trajectory_update',prefix+'. The old and updated splines use the same absolute touchdown time. Curves are contact-reference positions; clearance uses 19 mm foot spheres. Shading shows the configured planning allowance only in the middle 50% noncontact interval after instantiation, not a measured or proven tracking envelope.')
    # Continuous curves here are dense diagnostic evaluation. The actual certificate is the MINVO control inequality.
    lo=max(e['now'],e['start']+.25*duration);hi=e['start']+.75*duration
    tt=np.linspace(lo,hi,4001)
    co=clearance(evaluate(e['old'],tt)+[0,0,.019],e['height'])
    cn=clearance(evaluate(e['new'],tt)+[0,0,.019],e['height'])
    fig,(ax,)=panels(1,'Continuous-time collision safety')
    fig.text(.98,.035,prefix,ha='right',color='.35',fontsize=8)
    ax.plot((tt-e['start'])/duration,co,color=BLUE,label='Original clearance')
    ax.plot((tt-e['start'])/duration,cn,color=ORANGE,label='Updated clearance')
    sparse=np.array([e['start']+.25*duration,e['start']+.5*duration,e['start']+.75*duration]);sparse=sparse[sparse>=lo]
    cs=clearance(evaluate(e['old'],sparse)+[0,0,.019],e['height'])
    ax.scatter((sparse-e['start'])/duration,cs,color=GREEN,s=24,zorder=5,label='Sparse checks (0.25 phase spacing)')
    ax.fill_between((tt-e['start'])/duration,co,0,where=co<0,color=BLUE,alpha=.18)
    ax.axhline(0,color='black',lw=1.2,label='Collision boundary')
    ax.set(xlabel='Normalized swing time',ylabel='Signed sphere clearance (m)');ax.legend(loc='best')
    save(fig,out/'fig3_continuous_safety',prefix+'. Dense curve evaluation is an independent diagnostic, not the continuous-time certificate. Sparse check spacing is fixed at 0.25 swing phase; no claim of missed collision if any marker is negative.')
    fig,axs=panels(3,'Terminal boundary preservation at fixed touchdown time')
    fig.text(.98,.025,prefix,ha='right',color='.35',fontsize=8)
    names=['Position z (m)','Velocity z (m/s)','Acceleration z (m/s²)']
    end_res=[]
    for order,ax in enumerate(axs):
        before=evaluate(e['old'],t,order)[:,2];after=before.copy();after[active]=evaluate(e['new'],t[active],order)[:,2]
        ax.plot(tau,before,'--',color=BLUE,label='Original boundary reference')
        ax.plot(tau,after,color=GREEN,label='Boundary-preserving update')
        terminal=evaluate(e['old'],[e['end']],order)[0,2]
        ax.axhline(terminal,color='.5',ls=':',lw=.8);ax.axvline(duration,color='.3',ls=':',lw=1)
        ax.scatter([duration],[after[-1]],s=22,color=GREEN,zorder=4);ax.set_ylabel(names[order])
        end_res.append(float(np.linalg.norm(evaluate(e['new'],[e['end']],order)-evaluate(e['old'],[e['end']],order))))
    axs[0].legend(loc='upper right');axs[-1].set_xlabel('Time since lift-off (s)')
    save(fig,out/'fig4_terminal_boundary',prefix+f'. Original and updated terminal p/v/a residual norms: {end_res}. The dashed reference is not an unconstrained ablation.',3)
    fig,axs=panels(3,'State matching at actual activation')
    fig.text(.98,.025,prefix,ha='right',color='.35',fontsize=8)
    for order,ax in enumerate(axs):
        before=evaluate(e['old'],t,order)[:,2]
        ax.plot(tau,before,color=BLUE,label='Old reference')
        if 'planned' in e:
            ok=t>=max(e['planned']['knots'][0],e['now']);ax.plot(tau[ok],evaluate(e['planned'],t[ok],order)[:,2],color=ORANGE,label='Received candidate (unaligned)')
        ax.plot(tau[active],evaluate(e['new'],t[active],order)[:,2],color=GREEN,label='Matched update')
        ax.axvline(ta,color=BLUE,ls=':',lw=1.2)
        boundary=evaluate(e['new'],[e['now']],order)[0,2]
        ax.scatter([ta],[boundary],s=22,color='black',zorder=5)
        ax.set_ylabel(names[order])
    axs[0].legend(loc='upper right');axs[-1].set_xlabel('Time since lift-off (s)')
    save(fig,out/'fig5_state_splicing',prefix+'. Black points denote the selected activation boundary. Matching measured state does not imply old/new reference C2 continuity across the switch. Acceleration source is last valid allocated WBC input in Gazebo; old reference acceleration is a declared controlled input in the offline experiment.',3)
    boundaries=([np.array(e['boundary_position_centre'])-[0,0,.019],
                 np.array(e['boundary_velocity']),np.array(e['boundary_acceleration'])]
                if e.get('physical_commit') else [evaluate(e['old'],[e['now']],k)[0] for k in range(3)])
    boundary_residuals=[float(np.linalg.norm(evaluate(e['new'],[e['now']],k)[0]-boundaries[k])) for k in range(3)]
    report=dict(physical_commit=e.get('physical_commit',False),start=e['start'],activation=e['now'],end=e['end'],
        old_min_clearance_m=float(co.min()),new_min_clearance_m=float(cn.min()),
        sparse_all_safe=bool(np.all(cs>=0)),terminal_residuals=end_res,activation_residuals=boundary_residuals)
    (SOURCE_OUTPUT/'trajectory_metrics.json').write_text(json.dumps(report,indent=2)+'\n')
    with (SOURCE_OUTPUT/'trajectory_source.csv').open('w') as f:
        series=[];fields=['time_s']
        for key in ('old','planned','new'):
            if key not in e:continue
            for order,quantity in enumerate(('p','v','a')):
                values=evaluate(e['old'],t,order)
                use=t>=e[key]['knots'][0]
                values[use]=evaluate(e[key],t[use],order)
                series.append(values);fields.extend(f'{key}_{quantity}_{axis}' for axis in 'xyz')
        w=csv.writer(f);w.writerow(fields);w.writerows(np.column_stack((t,*series)))


def read_commits(trial):
    events=[]
    text=trial.with_suffix('.launch.log').read_text(errors='replace')
    for line in text.splitlines():
        if 'SWING_EVIDENCE ' not in line:continue
        e=json.loads(line.split('SWING_EVIDENCE ',1)[1]);e['physical_commit']=True
        manifest=json.loads(trial.with_suffix('.manifest.json').read_text());o=manifest['trial.obstacle']
        e['height']=o['pose'][2]+o['collision_box_size'][2]/2
        e['trial']=str(trial)
        events.append(e)
    return events


def timing_figure(trials,out,helper):
    # Independent units are runs; each swing is shown as a within-run observation.
    events=[];selection=[]
    for run_index,trial in enumerate(trials):
        data=rows(trial.with_name(trial.stem+'_wbc.csv'))
        detected=helper.touchdown_events(data)
        time=np.array([float(r['sim_time']) for r in data]);modes=np.array([int(float(r['mode'])) for r in data])
        total=0;gaps=0
        for bit in (8,4,2,1):
            stance=(modes&bit)!=0;ix=np.flatnonzero(~stance[:-1]&stance[1:])+1
            total+=len(ix);gaps+=int(np.sum(time[ix]-time[ix-1]>.025))
        selection.append(dict(trial=trial.name,native_rows=len(data),observed_swing_to_stance=total,
            mode_gap_over_25ms=gaps,without_qualifying_force_onset=total-gaps-len(detected),plotted=len(detected)))
        for e in detected:
            e.update(run=run_index,trial=trial.name);events.append(e)
    if not events:raise ValueError('No measured contact onsets: timing figure unavailable')
    fig,(ax,)=panels(1,'Fixed touchdown timing: measured contact observations')
    x=np.array([e['timing_ms'] for e in events]);y=np.arange(1,len(events)+1)
    ax.axvspan(-10,10,color=BLUE,alpha=.10,label='Displayed ±10 ms tolerance')
    ax.axvline(0,color='black',ls='--',lw=1.2,label='Scheduled contact switch')
    ax.scatter(x,y,c=BLUE,s=16,label='Measured fixed-timing contact')
    ax.set(xlabel='Detected contact minus scheduled switch (ms)',ylabel='Swing observation ID')
    ax.legend(loc='best')
    save(fig,out/'fig2_fixed_touchdown_timing',f'Gazebo: {len(events)} detected force onsets from {len(trials)} independent launches. Threshold 3 N, event window [-100,+80] ms, nearest onset. Scheduled time bracket midpoint from mode samples; bracket gaps >25 ms excluded. Event selection is a contact proxy, not proof of legal target contact. All available events are plotted; no variable-timing method is fabricated.')
    with (SOURCE_OUTPUT/'touchdown_source.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(events[0]));w.writeheader();w.writerows(events)
    (SOURCE_OUTPUT/'touchdown_selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    return events


def execution_figure(trial,e,out,geometry):
    data=rows(trial.with_name(trial.stem+'_wbc.csv'));leg=LEGS[e['leg']]
    controller=np.array([float(r['sim_time']) for r in data]);ros=np.array([float(r['ros_time']) for r in data]);
    mask=(controller>=max(e['now'],e['start']+.25*(e['end']-e['start'])))&(controller<=e['start']+.75*(e['end']-e['start']))
    if mask.sum()<3:raise ValueError('Too few execution samples in committed noncontact interval')
    ts=controller[mask];rr=ros[mask];stamps,feet=geometry.true_feet(trial.with_name(trial.stem+'_gz_poses.csv'))
    assert np.all(np.diff(stamps)>0), 'Truth timestamps must increase'
    ix=np.searchsorted(stamps,rr);ix=np.clip(ix,1,len(stamps)-1)
    if np.max(stamps[ix]-stamps[ix-1])>.020:raise ValueError('Truth pose interpolation gap >20ms')
    truth=np.column_stack([np.interp(rr,stamps,feet[:,e['leg'],k]) for k in range(3)])
    desired=evaluate(e['new'],ts)+[0,0,.019]
    boxes=geometry.world_boxes(trial.with_suffix('.world.sdf'))
    cr=geometry.signed_clearance(desired,boxes);ca=geometry.signed_clearance(truth,boxes)
    error=np.linalg.norm(truth-desired,axis=1)
    fig,axs=panels(2,'Reference clearance and actual foot execution')
    t=ts-e['start'];axs[0].plot(t,cr,color=BLUE,label='Activated reference')
    axs[0].plot(t,ca,color=ORANGE,label='Gazebo foot truth')
    axs[0].axhline(0,color='black',lw=1.2,label='Collision boundary');axs[0].set_ylabel('Signed clearance (m)')
    axs[0].legend(loc='upper right')
    axs[1].plot(t,error,color=ORANGE,label='Truth-to-reference position error')
    axs[1].axhline(.02,color=BLUE,ls='--',label='Planning tracking allowance (20 mm)')
    axs[1].set(ylabel='Position error (m)',xlabel='Time since lift-off (s)');axs[1].legend(loc='upper right')
    save(fig,out/'fig6_reference_execution',f'Gazebo {leg}: sphere radius 19 mm, exact world boxes. Truth link poses composed with robot pose; interpolated only across <=20 ms. Reference time uses each WBC sample controller timestamp. One HOCBF-enabled run; no unsupported HOCBF ablation. Planning allowance is a configured budget, not a proven error tube. Observed min actual clearance={ca.min():.8g} m, max error={error.max():.8g} m.',2)
    with (SOURCE_OUTPUT/'execution_source.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['controller_time_s','ros_time_s','reference_clearance_m','true_clearance_m','tracking_error_m']);w.writerows(np.column_stack((ts,rr,cr,ca,error)))


def sampled_execution_figure(trial,out,geometry):
    data=rows(trial.with_name(trial.stem+'_wbc.csv'))
    ct=np.array([float(r['sim_time']) for r in data]);ros=np.array([float(r['ros_time']) for r in data])
    modes=np.array([int(float(r['mode'])) for r in data])
    candidates=[]
    for leg_index,(leg,bit) in enumerate(zip(LEGS,(8,4,2,1))):
        swinging=(modes&bit)==0
        starts=np.flatnonzero(~swinging[:-1]&swinging[1:])+1
        ends=np.flatnonzero(swinging[:-1]&~swinging[1:])+1
        for first in starts:
            later=ends[ends>first]
            if not len(later):continue
            last=later[0];span=ct[last]-ct[first]
            mask=(ct>=ct[first]+.25*span)&(ct<=ct[first]+.75*span)
            ix=np.flatnonzero(mask)
            if len(ix)<4:continue
            x=np.array([float(data[j][leg+'_pd_x']) for j in ix])
            candidates.append((float(np.min(np.abs(x+1.2))),leg_index,leg,ix,ct[first]))
    if not candidates:raise ValueError('No complete swing to evaluate')
    _,li,leg,ix,start=min(candidates,key=lambda c:c[0])
    stamps,feet=geometry.true_feet(trial.with_name(trial.stem+'_gz_poses.csv'))
    assert np.all(np.diff(stamps)>0), 'Truth timestamps must increase'
    interp=np.searchsorted(stamps,ros[ix]);interp=np.clip(interp,1,len(stamps)-1)
    valid=(stamps[interp]-stamps[interp-1])<=.020
    ix=ix[valid]
    if len(ix)<4:raise ValueError('Truth unavailable in selected swing')
    truth=np.column_stack([np.interp(ros[ix],stamps,feet[:,li,k]) for k in range(3)])
    desired=np.array([[float(data[j][f'{leg}_pd_{axis}']) for axis in 'xyz'] for j in ix])
    measured=np.array([[float(data[j][f'{leg}_p_{axis}']) for axis in 'xyz'] for j in ix])
    boxes=geometry.world_boxes(trial.with_suffix('.world.sdf'))
    cr=geometry.signed_clearance(desired,boxes);ca=geometry.signed_clearance(truth,boxes)
    error=np.linalg.norm(truth-desired,axis=1);estimated=np.linalg.norm(measured-desired,axis=1)
    fig,axs=panels(2,'Reference clearance and actual foot execution')
    time=ct[ix]-start
    axs[0].plot(time,cr,color=BLUE,label='Executed reference samples')
    axs[0].plot(time,ca,color=ORANGE,label='Gazebo foot truth')
    axs[0].axhline(0,color='black',lw=1.2,label='Collision boundary');axs[0].set_ylabel('Signed clearance (m)');axs[0].legend(loc='upper right')
    axs[1].plot(time,error,color=ORANGE,label='Truth-to-reference error')
    axs[1].plot(time,estimated,color=GREEN,label='Estimated tracking error')
    axs[1].axhline(.02,color=BLUE,ls='--',label='Planning allowance (20 mm)')
    axs[1].set(ylabel='Position error (m)',xlabel='Time since lift-off (s)');axs[1].legend(loc='upper right')
    save(fig,out/'fig6_reference_execution',f'Gazebo {leg}; selected the complete swing whose noncontact reference passes closest to the obstacle centre (minimum |x+1.2|), irrespective of outcome. Shows its middle 50% noncontact window, all native WBC samples with <=20ms true-pose interpolation brackets. Desired and measured positions are foot-centre coordinates. Signed distance uses the actual world and 19mm foot sphere. This is execution evidence, not a committed online-repair example. min actual clearance={ca.min():.8g}m; min reference={cr.min():.8g}m; max true error={error.max():.8g}m. HOCBF is enabled but no causal HOCBF comparison is made.',2)
    with (SOURCE_OUTPUT/'execution_source.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['controller_time_s','ros_time_s','reference_clearance_m','true_clearance_m','tracking_error_m','estimated_error_m']);w.writerows(np.column_stack((ct[ix],ros[ix],cr,ca,error,estimated)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mechanism',type=Path,required=True);p.add_argument('--trials',type=Path,nargs='*',default=[]);p.add_argument('--output',type=Path,required=True);p.add_argument('--source-output',type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    global SOURCE_OUTPUT
    SOURCE_OUTPUT=a.source_output or a.mechanism.parent/'analysis'/a.output.name
    SOURCE_OUTPUT.mkdir(parents=True,exist_ok=True)
    root=next(q for q in Path(__file__).resolve().parents if (q/'src/quadruped_ros2_control/controllers').is_dir())
    sys.path.insert(0,str(root/'src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test'))
    import analyze_online_trial as helper
    import analyze_true_geometry as geometry
    offline=[json.loads(line) for line in (a.mechanism/'mechanism_events.jsonl').read_text().splitlines()]
    commits=[]
    for trial in a.trials:commits.extend(read_commits(trial))
    (SOURCE_OUTPUT/'commits.json').write_text(json.dumps(commits,indent=2)+'\n')
    eligible=[e for e in commits if 'old' in e and event_score(e)[0]]
    if eligible:
        chosen=eligible[0];trajectory_figures(chosen,a.output);execution_figure(Path(chosen['trial']),chosen,a.output,geometry)
    elif offline:
        repairs=[e for e in offline if event_score(e)[0]]
        chosen=max(repairs,key=lambda e:(round(e['now']-e['reveal'],8),e['reveal'],-e['height'])) if repairs else offline[0];trajectory_figures(chosen,a.output)
        if a.trials:sampled_execution_figure(a.trials[0],a.output,geometry)
    else:raise ValueError('No accepted production-algorithm trajectory to plot')
    if a.trials:timing_figure(a.trials,a.output,helper)
    # If there is no physical repair, still expose actual clearance/error from a documented
    # swing using WBC desired trajectory; never relabel it as an activated update.
    print(json.dumps({'physical_commits':len(commits),'physical_collision_repairs':len(eligible),'offline_cases':len(offline)}))

if __name__=='__main__':main()
