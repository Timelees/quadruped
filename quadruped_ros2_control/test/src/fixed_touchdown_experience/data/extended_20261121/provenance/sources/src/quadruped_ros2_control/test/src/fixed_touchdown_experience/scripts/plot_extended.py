#!/usr/bin/env python3
"""Five evidence figures; Python backend, real data only, unavailable outcomes explicit."""
import argparse
import csv
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.ticker import FuncFormatter, NullFormatter
import numpy as np
from plot_figures import evaluate
sys.path.insert(0,str(Path.home()/'.codex/skills/nature-figure/scripts'))
from audit_panel_alignment import require_matplotlib_panel_alignment
BLUE,ORANGE,GREEN='#2676aa','#df8127','#31966c'
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],'font.size':8,'axes.titlesize':9,'axes.labelsize':8,
    'xtick.labelsize':7,'ytick.labelsize':7,'legend.fontsize':7,'pdf.fonttype':42,'ps.fonttype':42,
    'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'lines.linewidth':1.6})


def csvrows(path):return list(csv.DictReader(path.open()))
def columns(rows,key):return np.array([float(r[key]) if r[key] not in ('','None') else np.nan for r in rows])
def make(n,title,vertical=False):
    fig,axs=plt.subplots(n if vertical else 1,1 if vertical else n,figsize=(7.2,6.8 if vertical else 3.3))
    fig.subplots_adjust(left=.12 if vertical else .085,right=.97,bottom=.11 if vertical else .19,top=.90 if vertical else .78,
                        hspace=.62,wspace=.48)
    axs=np.atleast_1d(axs)
    fig.suptitle(title,y=.98,fontsize=11)
    for i,ax in enumerate(axs):
        ax.text(-.10,1.1,chr(97+i),weight='bold',transform=ax.transAxes,fontsize=9)
        ax.grid(alpha=.16)
    return fig,axs


def save(fig,path,caption):
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,json_out=path.with_suffix('.alignment.json'),tolerance_pt=1.5,
        gutter_tolerance_pt=1.5,require_panel_labels=True,strict=True)
    fig.savefig(path.with_suffix('.pdf'))
    fig.savefig(path.with_suffix('.svg'))
    fig.savefig(path.with_suffix('.png'),dpi=600)
    path.with_suffix('.caption.txt').write_text(caption+'\n')
    plt.close(fig)


def heatmap(root,out):
    rows=csvrows(root/'sweep/trials.csv');assert len(rows)==6400
    aggregate=[];matrices=[]
    for field in ['candidate_certified','instantiated']:
        matrix=np.zeros((8,8))
        for h in range(1,9):
            for b in range(1,9):
                rr=[r for r in rows if abs(float(r['height_m'])-.02*h)<1e-8 and abs(float(r['remaining_s'])-.05*b)<1e-8]
                assert len(rr)==100 and len(set(r['trial'] for r in rr))==100
                count=sum(int(r[field]) for r in rr);rate=count/100;matrix[h-1,b-1]=100*rate
                z=1.96;den=1+z*z/100;centre=(rate+z*z/200)/den
                radius=z*np.sqrt(rate*(1-rate)/100+z*z/40000)/den
                aggregate.append(dict(height_cm=2*h,remaining_ms=50*b,metric=field,n=100,success=count,
                    rate=rate,wilson95_low=max(0,centre-radius),wilson95_high=min(1,centre+radius)))
        matrices.append(matrix)
    with (root/'heatmap_aggregate.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=aggregate[0]);w.writeheader();w.writerows(aggregate)
    fig,axs=make(3,'Remaining-time feasibility | production-algorithm replay')
    fig.subplots_adjust(right=.89)
    titles=['Candidate certified','Boundary instantiation','On-time touchdown']
    for k,ax in enumerate(axs):
        if k<2:
            im=ax.imshow(matrices[k],origin='lower',vmin=0,vmax=100,cmap='viridis',aspect='auto',extent=[25,425,1,17])
            for i in range(8):
                for j in range(8):ax.text(50*(j+1),2*(i+1),f'{matrices[k][i,j]:.0f}',ha='center',va='center',fontsize=6,
                    color='.15' if matrices[k][i,j]>55 else 'white')
        else:
            ax.set_facecolor('.95');ax.text(.5,.54,'Not measured',ha='center',transform=ax.transAxes,color='.35')
            ax.text(.5,.36,'No physical trials per cell',ha='center',transform=ax.transAxes,fontsize=7,color='.35')
            ax.set(xlim=(25,425),ylim=(1,17))
        ax.set(title=titles[k],xlabel='Remaining swing time (ms)',xticks=[50,150,250,350],yticks=np.arange(2,17,2));ax.grid(False)
    axs[0].set_ylabel('Obstacle height (cm)')
    cax=fig.add_axes([.925,.22,.014,.53]);fig.colorbar(im,cax=cax,label='Pass rate (%)');cax.tick_params(labelsize=6)
    fig.text(.97,.035,'100 paired seeded states / cell; IK and WBC admission excluded',ha='right',fontsize=7,color='.35')
    save(fig,out/'fig7_feasibility_heatmap','8 heights × 8 remaining-time bins × 100 seeded local uncertainty states = 6400 production-code calls. Same 100 seeds paired across cells, not independent Gazebo trials. Source references are translated actual MPC swings; task.info is loaded without relaxation. Candidate certification and boundary instantiation are conditional algorithm gates; sampled IK, WBC preview, transport and physical touchdown are not evaluated by this replay. CIs: 95% Wilson in heatmap_aggregate.csv. Every failed request remains in the denominator. Physical panel is unavailable, not zero success.')


def runtime(root,out):
    rows=csvrows(root/'sweep/trials.csv');fig,axs=make(2,'Online replanning computation | replay and Gazebo')
    fields=['corridor_ms','admission_ms','instantiate_ms','total_ms'];labels=['Corridor','Admission\n(QP + checks)','Instantiate','End-to-end']
    values=[columns([r for r in rows if k!='instantiate_ms' or int(r['candidate_certified'])],k) for k in fields]
    stats={k:dict(n=len(v),p50=float(np.quantile(v,.5)) if len(v) else None,
        p95=float(np.quantile(v,.95)) if len(v) else None,p99=float(np.quantile(v,.99)) if len(v) else None) for k,v in zip(fields,values)}
    online=json.loads((root/'gazebo_pilot.replans.json').read_text())['decisions']
    online_times=np.array([r['compute_ms'] for r in online])
    assert len(online_times)>0
    stats['gazebo_online_replan_ms']=dict(n=len(online_times),p50=float(np.quantile(online_times,.5)),p95=float(np.quantile(online_times,.95)),p99=float(np.quantile(online_times,.99)))
    with (root/'gazebo_online_latency.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(online[0]));writer.writeheader();writer.writerows(online)
    (root/'runtime_quantiles.json').write_text(json.dumps(stats,indent=2)+'\n')
    x=np.arange(4);med=[stats[k]['p50'] or 0 for k in fields];high=[stats[k]['p95'] or 0 for k in fields]
    axs[0].bar(x,med,color=BLUE,width=.55,label='P50')
    axs[0].scatter(x,high,color=ORANGE,marker='_',s=130,label='P95',zorder=4)
    axs[0].set(xticks=x,xticklabels=labels,ylabel='Computation time (ms)')
    axs[0].legend(loc='lower left',bbox_to_anchor=(0,1.03),frameon=False,ncol=2)
    groups=[columns([r for r in rows if lo<=float(r['height_m'])<=hi+1e-8],'total_ms') for lo,hi in [(.02,.06),(.08,.12),(.14,.16)]]
    groups.append(online_times)
    axs[1].boxplot(groups,labels=['Replay\n2–6 cm','Replay\n8–12 cm','Replay\n14–16 cm','Gazebo\nonline'],showfliers=True,
        flierprops=dict(marker='.',markersize=2,alpha=.35),medianprops=dict(color=ORANGE))
    axs[1].axhline(5,color='.4',ls='--',lw=.8,label='WBC period: 5 ms')
    axs[1].set(ylabel='End-to-end time (ms)');axs[1].legend(loc='lower left',bbox_to_anchor=(0,1.03),frameon=False)
    assert all(np.all(g>0) for g in groups),'Log time axis requires positive measurements'
    axs[1].set_yscale('log')
    axs[1].yaxis.set_major_formatter(FuncFormatter(lambda value,position:f'{value:g}'))
    axs[1].yaxis.set_minor_formatter(NullFormatter())
    fig.text(.97,.035,f"End-to-end P50 / P95 / P99: {stats['total_ms']['p50']:.3f} / {stats['total_ms']['p95']:.2f} / {stats['total_ms']['p99']:.2f} ms",ha='right',fontsize=7,color='.35')
    save(fig,out/'fig8_replanning_latency','Wall-clock replay measurements on this host, including all failures and early exits; sequential CPU run. Corridor construction, whole production admission (QP and candidate certification combined), boundary instantiation and directly measured end-to-end time. Instantiation percentiles include only candidate-passing requests; an empty stage is explicitly absent. Map acquisition and MRT residence are excluded; these are not WBC timings or a real-time deadline guarantee. P50/P95/P99 and n in runtime_quantiles.json; end-to-end percentiles are measured directly, never sums of stage quantiles. The fourth box is 390 actual Gazebo online replan log durations, kept separate from the three replay groups; no actual update was committed in this pilot. Its exact n and quantiles are saved, not pooled with replay timings. Box: quartiles and median; whiskers 1.5 IQR; dots all outliers; logarithmic time axis in panel b.')


def hull(points):
    pts=sorted(set(map(tuple,points)))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[];upper=[]
    for p in pts:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return np.array(lower[:-1]+upper[:-1])
def area(p):return abs(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1)))/2


def hull_figure(root,out):
    segments=[json.loads(s) for name in ['hulls.jsonl','hulls_instantiated.jsonl'] for s in (root/'sweep'/name).read_text().splitlines()];metrics=[]
    for i,s in enumerate(segments):
        b=hull(np.array(s['bernstein'])[:,[0,2]]);m=hull(np.array(s['minvo'])[:,[0,2]])
        ab,am=area(b),area(m)
        metrics.append(dict(index=i,source=s['source'],kind='offline_instantiated' if 'parent_index' in s else 'Gazebo_MPC',start=s['start'],end=s['end'],bernstein_area=ab,minvo_area=am,
                            reduction=1-am/ab if ab>1e-10 else -np.inf))
    valid=[r for r in metrics if np.isfinite(r['reduction'])];best=max(valid,key=lambda r:r['bernstein_area']-r['minvo_area']);s=segments[best['index']]
    with (root/'hull_metrics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=metrics[0]);w.writeheader();w.writerows(metrics)
    (root/'selected_hull.json').write_text(json.dumps(dict(segment=s,metrics=best,selection='Maximum absolute XZ area reduction among all full polynomial spans of logged MPC references and all admitted offline instances with Bernstein area >1e-10 m²; no span reshaping or arbitrary subinterval search.'),indent=2)+'\n')
    fig,axs=make(2,'MINVO and Bernstein | identical physical polynomial span')
    endpoints=evaluate(s['spline'],[s['start'],s['end']])[:,[0,2]]
    x0,z0=endpoints[0];slope=(endpoints[1,1]-z0)/(endpoints[1,0]-x0)
    def display(points):
        p=np.asarray(points)[:,[0,2]].copy();p[:,1]-=z0+slope*(p[:,0]-x0);p[:,0]-=x0
        return p*1000
    (root/'hull_display_transform.json').write_text(json.dumps(dict(x0=x0,z0=z0,chord_slope=slope,scale_to_mm=1000,shear_determinant=1),indent=2)+'\n')
    for key,color,label in [('bernstein',ORANGE,'Bernstein'),('minvo',BLUE,'MINVO')]:
        points=display(s[key]);p=hull(points)
        axs[0].add_patch(Polygon(p,fc=color,ec=color,alpha=.16,lw=1.3,label=label))
        axs[0].scatter(points[:,0],points[:,1],color=color,s=15,zorder=3)
    t=np.linspace(s['start'],s['end'],601);curve=evaluate(s['spline'],t)
    shown=display(curve)
    axs[0].plot(shown[:,0],shown[:,1],color='.15',label='Same quintic curve')
    direction_rows=[]
    for theta in np.linspace(0,2*np.pi,361):
        normal=np.array([np.cos(theta),np.sin(theta)]);offset=float(np.max(curve[:,[0,2]]@normal))+.001
        direction_rows.append(dict(angle_rad=theta,offset=offset,bernstein_slack=offset-float(np.max(np.array(s['bernstein'])[:,[0,2]]@normal)),minvo_slack=offset-float(np.max(np.array(s['minvo'])[:,[0,2]]@normal))))
    with (root/'hull_direction_slack.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=direction_rows[0]);w.writeheader();w.writerows(direction_rows)
    axs[0].autoscale_view();axs[0].set(xlabel='Segment-relative x (mm)',ylabel='Z minus segment chord (mm)')
    axs[0].legend(loc='lower left',bbox_to_anchor=(0,1.02),frameon=False,ncol=2)
    vals=np.array([best['bernstein_area'],best['minvo_area']])*1e4
    axs[1].grid(False)
    axs[1].barh([1,0],vals,color=[ORANGE,BLUE],height=.55)
    for y,v in zip([1,0],vals):axs[1].text(v+.025*max(vals),y,f'{v:.2f}',va='center',fontsize=8)
    axs[1].set(yticks=[1,0],yticklabels=['Bernstein','MINVO'],xlabel='XZ hull area (cm²)',xlim=(0,max(vals)*1.3),
        title=f"Area reduction {100*best['reduction']:.1f}%")
    save(fig,out/'fig9_minvo_bernstein','Both control hulls come from the same physical quintic span of either a logged MPC reference or an admitted simulation-configured offline instance (kind recorded in selected_hull.json) and the production extractPolynomialSegments/hullMap functions. Best full span selected by maximum absolute XZ area reduction; all candidate metrics saved. Panel a subtracts the segment chord (an invertible affine shear with determinant one) and uses millimetres to make hull thickness visible; all area values remain those of the original XZ projection. The exact display transform is saved. Areas are planar, not 3D volumes. This illustrates reduced geometric conservatism for this span; it does not imply superior margin for every half-space direction. All controls and source curve preserved in selected_hull.json. Directional slack for 361 diagnostic support half-spaces (same dense curve support +1 mm) is in hull_direction_slack.csv; these diagnostic offsets are not measured obstacle boundaries.')


def inputs(root,out):
    selected=csvrows(root/'selected_control.csv');bounds=csvrows(root/'input_bounds.csv');snap=json.loads((root/'input_slice.json').read_text());sw=json.loads((root/'selected_swing.json').read_text());start=sw['start']
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.4));fig.subplots_adjust(left=.10,right=.97,bottom=.12,top=.87,hspace=.78,wspace=.48)
    fig.suptitle('Whole-body input feasibility | Gazebo controller data',y=.98,fontsize=11)
    for i,ax in enumerate(axs.flat):ax.text(-.12,1.1,chr(97+i),weight='bold',transform=ax.transAxes);ax.grid(alpha=.16)
    ax=axs[0,0];t=columns(selected,'time')-start;tb=columns(bounds,'time')-start
    ax.fill_between(tb,columns(bounds,'z_min'),columns(bounds,'z_max'),color=BLUE,alpha=.18,label='Physical WBC projection')
    ax.plot(t,columns(selected,'uz'),color=GREEN,label='Allocated input');ax.plot(t,columns(selected,'nominal_z'),ls='--',color=ORANGE,label='Nominal target')
    ax.set(xlabel='Swing time (s)',ylabel='Foot acceleration z (m/s²)')
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.01),frameon=False,ncol=1,fontsize=6)
    ax=axs[0,1]
    for key,color,label in [('physical',BLUE,'Physical WBC'),('joint',GREEN,'Joint safety')]:
        p=np.array(snap[key]['points'])
        if len(p):ax.add_patch(Polygon(p,fc=color,ec=color,alpha=.22,label=label))
    nominal=np.array(snap['nominal']);actual=np.array(snap['actual'])
    ax.scatter(nominal[0],nominal[2],marker='x',color=ORANGE,s=35,label='Nominal target');ax.scatter(actual[0],actual[2],color=GREEN,s=22,label='Allocated input')
    ax.autoscale_view();ax.set(xlabel='Foot acceleration x (m/s²)',ylabel='Foot acceleration z (m/s²)')
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.01),frameon=False,ncol=2,fontsize=6)
    ax=axs[1,0];ax.plot(t,columns(selected,'input_witness_ineq')/1e-5,color=BLUE,label='Inequality residual')
    ax.plot(t,columns(selected,'input_witness_eq')/1e-5,color=ORANGE,ls='--',label='Equality absolute residual')
    ax.axhline(1,color='.4',ls=':',lw=.8,label='Numerical tolerance');ax.set(xlabel='Swing time (s)',ylabel='Witness residual / tolerance')
    ax.legend(loc='lower left',bbox_to_anchor=(0,1.01),frameon=False,ncol=1,fontsize=6)
    ax=axs[1,1];ax.plot(t,columns(selected,'torque_usage'),color=GREEN);ax.axhline(1,color=ORANGE,ls='--',lw=.9)
    ax.set(xlabel='Swing time (s)',ylabel='Maximum net torque / 12 Nm')
    save(fig,out/'fig10_real_input_feasibility','Actual Gazebo WBC model snapshots, not hardware measurements. Physical set uses full rigid-body/contact/friction/torque rows, instantaneous motor bounds, fixed baseline body acceleration, contact-force reserve and swing correction reserve. Panel a is a 1D projection (necessary only). Panel b is a ux/uz projection at actual uy, with complete feasible WBC witnesses; adaptive LP support tolerance 1e-6, intersection adds ALL exported safety rows including other legs and available tube/landing constraints. Empty joint set is retained as empty. Panel c reports complete WBC witness residuals in native mixed constraint units, NOT a falsely inferred 3D foot-only polytope residual. Positive residual beyond 1e-5 falsifies membership of this witness; passing is conditional on the model. Panel d uses net model torque before hardware PD compensation, 12 Nm source bound. Actual actuator mismatch and uncertainty calibration are separate requirements. Selection is fixed by selected_swing.json / input_slice.json; failures are retained.')


def hocbf(root,out):
    rows=csvrows(root/'selected_control.csv');sw=json.loads((root/'selected_swing.json').read_text());t=columns(rows,'time')-sw['start'];fig,axs=make(3,'HOCBF safety functions and robust hold residual | Gazebo',True)
    for ax,key,label,color in zip(axs,['h_min','psi1_min','R_min'],['Robust h (m)','Robust ψ₁ (m/s)','Hold residual (m/s²)'],[BLUE,GREEN,'#7654a3']):
        ax.fill_between(t,0,columns(rows,key),color=color,alpha=.10)
        ax.plot(t,columns(rows,key),color=color,marker='.',markersize=3);ax.axhline(0,color='.45',ls='--',lw=.9)
        failed=np.array([int(r['status']) not in [1,13,15,16] for r in rows]);
        ax.scatter(t[failed],columns(rows,key)[failed],facecolors='none',edgecolors='#ac3434',s=24,zorder=4,label='Safety entry / solve failed')
        ax.set(ylabel=label,xlabel='Swing time (s)',xlim=(0,sw['end']-sw['start']));ax.legend(loc='lower left',bbox_to_anchor=(0,1.02),frameon=False)
    save(fig,out/'fig11_hocbf_residual','Selected swing is shown on its original time axis; intervals with no active diagnostic planes are unobserved, not zero. All exported plane states including entry and solve failures are retained. Minima across active geometric planes are envelopes and need not share the minimizing plane. Per-plane values and normals/offsets are in hocbf_planes.csv. h subtracts 1 mm state error, hdot subtracts 0.01 m/s, disturbance support uses 0.5 m/s² and gains 20/20. Panel c is the minimum of all three Bernstein coefficients for a 6 ms zero-order hold, including 20×0.006 sample reserve. Allocated acceleration is computed from full WBC q and Jq+Jdot v; model-valid status is recorded and does not prove applied actuator equivalence. Nonnegative samples cannot prove an uncalibrated global disturbance budget or continuous executed safety.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    heatmap(a.directory,a.output);runtime(a.directory,a.output);hull_figure(a.directory,a.output);inputs(a.directory,a.output);hocbf(a.directory,a.output)

if __name__=='__main__':main()
