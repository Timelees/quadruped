#!/usr/bin/env python3
"""Paper plots from production exports and retained Gazebo trials (no invented data).

Python/numpy/matplotlib are already provided by the ROS workstation.
Use --audit-tools to retain the optional render-time panel-alignment manifest.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

P=argparse.ArgumentParser(description=__doc__)
P.add_argument('data',type=Path)
P.add_argument('--output',type=Path,required=True)
P.add_argument('--audit-tools',type=Path)
P.add_argument('--corridor-data', type=Path, help='Schema 2 corridor exports; other evidence still uses data')
a=P.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
if a.audit_tools:
    sys.path.insert(0,str(a.audit_tools))
    from audit_panel_alignment import require_matplotlib_panel_alignment
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Arial','Helvetica','DejaVu Sans'],'font.size':8,'axes.labelsize':8,
    'axes.titlesize':9,'legend.fontsize':7,'xtick.labelsize':7,'ytick.labelsize':7,
    'svg.fonttype':'none','pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,
    'axes.linewidth':.6,'lines.linewidth':1.2})
blue,orange,gray='#267699','#C47A35','#72777D'

def load(p):return np.atleast_1d(np.genfromtxt(p,delimiter=',',names=True,encoding=None))
def finish(fig,name,axes):
    for i,ax in enumerate(np.asarray(axes).flat):
        ax.text(-.18,1.08,chr(97+i),transform=ax.transAxes,fontweight='bold',fontsize=9)
    fig.subplots_adjust(left=.10,right=.98,bottom=.20,top=.86,wspace=.43,hspace=.55)
    fig.canvas.draw()
    if a.audit_tools:
        require_matplotlib_panel_alignment(fig,json_out=a.output/(name+'.alignment.json'),
            tolerance_pt=1.5,gutter_tolerance_pt=1.5,strict=True)
    # Fixed physical size, editable text in both vector formats.
    fig.savefig(a.output/(name+'.pdf'))
    fig.savefig(a.output/(name+'.svg'))
    fig.savefig(a.output/(name+'.png'),dpi=600)
    plt.close(fig)

# Same per-scene plots and palette as the standalone corridor plotting command.
from plot_corridor_sections import SCENES, plot_scene
for scene in SCENES:
    plot_scene(a.corridor_data or a.data, a.output, scene, audit_tools=a.audit_tools)

# Comparison question: do continuous hull constraints avoid the collisions
# missed by sparse collocation, and how much tighter is the MINVO bound?
m=load(a.data/'minvo/metrics.csv'); h=load(a.data/'minvo/hull_bounds.csv')
fig,axs=plt.subplots(1,2,figsize=(7.2,2.9))
counts=[]
for mode in range(3):
    s=m[(m['mode']==mode)&(m['repeat']==0)]
    # Repeats are timing measurements of identical geometry, not new cases.
    safe=int(sum((s['solved']==1)&(s['certified']==1)))
    unsafe=int(sum((s['solved']==1)&(s['certified']==0)))
    counts.append([safe,unsafe,len(s)-safe-unsafe])
c=np.array(counts);bottom=np.zeros(3)
for i,(label,color,hatch) in enumerate([('Certified',blue,''),('Uncertified',orange,'//'),('Unsolved','#D2D5D8','..')]):
    axs[0].bar(np.arange(3),c[:,i],bottom=bottom,color=color,hatch=hatch,label=label,width=.6)
    bottom+=c[:,i]
axs[0].set(xticks=range(3),xticklabels=['Discrete','Bézier','MINVO'],ylabel='Geometries (n = 27)',ylim=(0,float(bottom.max())+7),title='Independent collision certificate')
axs[0].legend(loc='upper center',fontsize=6.5,ncol=3,columnspacing=.9,handlelength=1.3)
axs[1].plot(h['true_min']*1000,h['true_min']*1000,color=gray,ls=':',label='Exact minimum')
axs[1].plot(h['true_min']*1000,h['bezier_lower']*1000,color=orange,ls='--',label='Bézier lower bound')
axs[1].plot(h['true_min']*1000,h['minvo_lower']*1000,color=blue,label='MINVO lower bound')
axs[1].axhline(0,color=gray,lw=.5)
axs[1].set(xlabel='Exact curve minimum (mm)',ylabel='Hull lower bound (mm)',title='Paired conservatism check')
axs[1].legend(loc='upper left')
finish(fig,'minvo_comparison',axs)

# Calibration is a separate measurement, not a trajectory-method ablation.
stability=a.data.parent/'stability'
fig,ax=plt.subplots(figsize=(3.6,2.8))
calibration=[]
calibration_runs=[('stairs_map_91','Previous pipeline',orange,'--'),('stairs_camera_96','Corrected pipeline',blue,'-')]
calibration_data={stem:load(stability/(stem+'_map.csv')) for stem,_,_,_ in calibration_runs}
calibration_data={stem:v[(v['ros_time']>=5)&(v['ros_time']<=12)] for stem,v in calibration_data.items()}
sites=sorted(set.intersection(*(set(v['site']) for v in calibration_data.values())))
for stem,label,color,style in calibration_runs:
    v=calibration_data[stem]
    xs=[];ys=[]
    for site in sites:
        s=v[v['site']==site]
        xs.append(s['true_surface'][0]*1000);ys.append(np.median(s['median_elevation']-s['true_surface'])*1000)
        calibration.append(dict(pipeline=label,surface_mm=xs[-1],median_error_mm=ys[-1],map_samples=len(s)))
    ax.plot(xs,ys,marker='o',color=color,ls=style,label=label)
ax.axhline(0,color=gray,lw=.6)
ax.set(xlabel='True tread height (mm)',ylabel='Median map error (mm)',title='Measured map calibration')
ax.set_xticks(sorted({row['surface_mm'] for row in calibration}))
ax.legend(loc='upper left');fig.subplots_adjust(left=.20,right=.98,bottom=.20,top=.86)
fig.savefig(a.output/'map_calibration.pdf')
fig.savefig(a.output/'map_calibration.svg')
fig.savefig(a.output/'map_calibration.png',dpi=600)
plt.close(fig)
(a.output/'calibration_source.json').write_text(json.dumps(calibration,indent=2)+'\n')

# Diagnose support loss at the first scheduled RF stance on the second tread.
# This is a retained failure and a final run, not a single-variable ablation.
fig,axs=plt.subplots(2,2,figsize=(7.2,4.7))
support_events=[]
support_gaps=[];support_forces=[]
for col,(stem,label,color) in enumerate([
        ('stairs_repeat_89','Earlier failed run',orange),
        ('stairs_final_104','Final configuration',blue)]):
    w=load(stability/(stem+'_wbc.csv'))
    starts=np.flatnonzero((w['mode'][1:]==6)&(w['mode'][:-1]!=6))+1
    starts=starts[(w['RF_p_x'][starts]>2.55)&(w['RF_p_x'][starts]<2.85)]
    if not len(starts):raise RuntimeError('Missing second-tread RF stance event: '+stem)
    t0=w['ros_time'][starts[0]]
    v=w[(w['ros_time']>=t0-.10)&(w['ros_time']<=t0+.35)]
    dt=v['ros_time']-t0
    # Feet are estimated world-frame kinematics; .2 m tread + .019 m radius.
    gap=(v['RF_p_z']-.219)*1000
    support_gaps.extend(gap);support_forces.extend(v['sensor_RF'])
    axs[0,col].plot(dt,gap,color=color)
    axs[1,col].plot(dt,v['sensor_RF'],color=color)
    axs[0,col].set(title=label,ylabel='Estimated RF gap (mm)')
    axs[1,col].set(xlabel='Time from planned stance (s)',ylabel='RF sensor force (N)')
    for ax in axs[:,col]:
        ax.axvspan(0,.35,color=gray,alpha=.08)
        ax.axvline(0,color=gray,ls=':',lw=.6)
        ax.set_xlim(-.10,.35)
    axs[0,col].axhline(0,color=gray,lw=.6)
    stance=v[(v['ros_time']>=t0)&(v['mode']==6)]
    support_events.append(dict(trial=stem,event_ros_time=float(t0),
        selection='First RF stance transition with estimated RF x in (2.55, 2.85) m',
        stance_median_gap_mm=float(np.median((stance['RF_p_z']-.219)*1000)),
        stance_median_sensor_N=float(np.median(stance['sensor_RF']))))
for col in range(2):
    axs[0,col].set_ylim(min(-5,float(np.min(support_gaps))-2),np.ceil(max(support_gaps)/20)*20+5)
    axs[1,col].set_ylim(0,np.ceil(max(support_forces)/10)*10+5)
finish(fig,'support_diagnosis',axs)
(a.output/'support_source.json').write_text(json.dumps(support_events,indent=2)+'\n')

summary=json.loads((a.data/'stability_summary.json').read_text())
trials=summary['final_trials']
if trials:
    fig,axs=plt.subplots(2,2,figsize=(7.2,4.7))
    for col,scenario in enumerate(['stairs','platform']):
        selected=[r for r in trials if r['scenario']==scenario]
        for i,r in enumerate(selected):
            # Use the complete retained truth series after initialization. No smoothing.
            path=stability/(r['trial']+'.csv')
            v=np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding=None)
            v=v[np.isin(v['phase'],['forward','hold'])]
            x=v['x'] if scenario=='stairs' else v['y']
            for ax,y in [(axs[0,col],v['z']),(axs[1,col],v['roll']*180/np.pi)]:
                ax.plot(x,y,color=[blue,orange,gray][i%3],alpha=.85,lw=.8,label=f'Trial {i+1}')
        axs[0,col].set_title('10 cm stairs' if scenario=='stairs' else '12.5 cm left platform')
        axs[0,col].set_ylabel('Base height (m)');axs[1,col].set_ylabel('Roll angle (deg)')
        axs[1,col].set_xlabel('Forward world x (m)' if scenario=='stairs' else 'Forward world y (m)')
        axs[0,col].legend(loc='upper left');axs[1,col].axhline(0,color='#bbb',lw=.5)
        axs[1,col].set_ylim(-4,4)
    finish(fig,'closed_loop_validation',axs)
print(a.output)
