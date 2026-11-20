"""Publication figures from observed or deterministic production experiment data."""
import csv
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from paper_experiments import read_csv, numeric, save_figure, scan_summary

plt.rcParams.update({'font.family': 'sans-serif', 'font.sans-serif': ['DejaVu Sans'], 'font.size': 7,
                     'pdf.fonttype': 42, 'svg.fonttype': 'none',
                     'axes.spines.top': False, 'axes.spines.right': False})
COLORS = ['#90979c', '#b68a54', '#7ba8ac', '#376b91']
METHODS = ['Frozen', 'Variable', 'Direct', 'Admitted']


def plot_state_boundary(root):
    rows = read_csv(root/'data/state_admission_scan.csv')
    observed_times = sorted(set(round(.4*(1-float(row['phase']))-float(row['delay_ms'])/1000,8) for row in rows))
    # Keep untested intermediate times blank, rather than stretch an irregular
    # categorical grid into a physical-time heatmap.
    times = np.round(np.arange(min(observed_times),max(observed_times)+.01,.02),8).tolist()
    shifts = sorted(set(float(row['position_shift_mm']) for row in rows))
    matrix = np.full((len(shifts),len(times)),np.nan)
    source = []
    for iy,shift in enumerate(shifts):
        for ix,time in enumerate(times):
            group = [row for row in rows if float(row['position_shift_mm'])==shift and
                abs(.4*(1-float(row['phase']))-float(row['delay_ms'])/1000-time)<1e-7]
            if group:
                count = sum(int(row['outcome'])==1 for row in group)
                matrix[iy,ix] = count/len(group)
                source.append([time,shift,len(group),count])
    figure,ax = plt.subplots(figsize=(7.2,2.8))
    figure.subplots_adjust(left=.12,right=.86,bottom=.25,top=.90)
    image = ax.imshow(matrix,origin='lower',aspect='auto',vmin=0,vmax=1,cmap='Blues')
    ax.set_xticks(range(len(times)),[f'{t*1000:g}' for t in times])
    ax.set_yticks(range(len(shifts)),[f'{s:g}' for s in shifts])
    ax.set_xlabel('Remaining time at activation (ms)')
    ax.set_ylabel('Activation state Z offset (mm)')
    figure.colorbar(image,ax=ax,label='Admission fraction (two velocity cases)')
    with (root/'data/state_heatmap_source.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['remaining_s','state_shift_mm','conditions','admitted']);writer.writerows(source)
    summary=dict(conditions=len(rows),admitted=sum(int(r['outcome'])==1 for r in rows),
        fixed_obstacle_height_m=.08,variable='activation state offset, not obstacle height',
        maximum_splice_mm=max(float(r['splice_mm']) for r in rows if int(r['outcome'])==1))
    (root/'data/state_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    save_figure(figure,root/'figure/state_boundary')


def plot_reveal(root, rows, name='reveal_comparison'):
    """Show local interventions; route completion cannot substitute for them."""
    figure, axes = plt.subplots(1,3,figsize=(7.2,2.9))
    figure.subplots_adjust(left=.09,right=.98,bottom=.26,top=.88,wspace=.7)
    methods = ('frozen','direct','admitted')
    phases = sorted({row['phase'] for row in rows})
    for index, method in enumerate(methods):
        for pindex, phase in enumerate(phases):
            group = [row for row in rows if row['phase']==phase and row['method']==method]
            if not group:
                continue
            x = pindex+(index-1)*.20
            count = sum(row['updates_in_triggered_swing']>0 for row in group)
            axes[0].bar(x, count,
                        width=.17,color=COLORS[index+1],label=method.title() if pindex==0 else None)
            axes[0].scatter(x,count,s=10,color=COLORS[index+1],zorder=3)
            for rindex, row in enumerate(group):
                for panel, key, scale in ((axes[1],'clearance_m',1000), (axes[2],'contact_error_ms',1)):
                    value = row.get(key)
                    if value is not None:
                        panel.scatter(x+(rindex-(len(group)-1)/2)*.035,value*scale,
                            s=13,color=COLORS[index+1],marker='o' if row['crossing'] else 'x')
    axes[0].set_ylabel('Trials with an in-swing activation')
    axes[1].set_ylabel('Post-reveal noncontact clearance (mm)')
    axes[2].set_ylabel('Measured LF load-onset error (ms)')
    axes[1].axhline(0,color='#90979c',linewidth=.7)
    axes[2].axhline(0,color='#90979c',linewidth=.7)
    axes[0].legend(frameon=False,loc='upper left',fontsize=6)
    repetitions=max([sum(r['phase']==phase and r['method']==method for r in rows)
                     for phase in phases for method in methods]+[1])
    axes[0].set_ylim(-.1,repetitions*1.55)
    axes[0].set_yticks(range(repetitions+1))
    for letter, ax in zip('abc',axes):
        ax.set_xticks(range(len(phases)),[f'{phase*100:g}' for phase in phases])
        ax.set_xlim(-.5,len(phases)-.5)
        ax.set_xlabel('Requested reveal phase (%)')
        ax.text(-.18,1.08,letter,transform=ax.transAxes,fontweight='bold')
    save_figure(figure,root/'figure'/name)


def plot_scan(root, boundary=False):
    rows=read_csv(root/'data/scan.csv')
    summary=scan_summary(rows)
    (root/'data/summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if boundary:
        # No interpolation or extrapolation: every heatmap cell is a complete
        # factorial of explicitly enumerated state offsets/delays at that T,h.
        selected=[row for row in rows if row['method']=='Admitted']
        frozen={row['case_id']:float(row['clearance_m']) for row in rows if row['method']=='Frozen'}
        observed=sorted(set(round(float(row['remaining_s']),8) for row in selected))
        times=np.round(np.arange(min(observed),max(observed)+.005,.01),8).tolist()
        heights=sorted(set(float(row['height_m']) for row in selected))
        admission=np.full((len(heights),len(times)),np.nan)
        repair=np.full_like(admission,np.nan)
        source=[]
        limits=[]
        for ix,t in enumerate(times):
            at_time=[]
            for iy,h in enumerate(heights):
                group=[row for row in selected if float(row['height_m'])==h and abs(float(row['remaining_s'])-t)<1e-7]
                if group:
                    accepted=[row for row in group if int(row['admitted'])]
                    collisions=[row for row in group if frozen[row['case_id']] < -1e-6]
                    repaired=[row for row in accepted if frozen[row['case_id']] < -1e-6 and float(row['clearance_m'])>=-1e-6]
                    admission[iy,ix]=len(accepted)/len(group)
                    if collisions:
                        repair[iy,ix]=len(repaired)/len(collisions)
                    source.append([t,h,len(group),len(accepted),len(collisions),len(repaired)])
                    at_time.extend(repaired)
            if t in observed:
                limits.append([t,len(at_time),max((float(r['height_m']) for r in at_time),default=None),
                    max((max(0.,-frozen[r['case_id']]) for r in at_time),default=None)])
        figure,axes=plt.subplots(2,1,figsize=(7.2,4.7))
        figure.subplots_adjust(left=.1,right=.85,bottom=.16,top=.92,hspace=.75)
        cmap=plt.get_cmap('Blues').copy()
        cmap.set_bad('#e4e4e4')
        for label,ax,matrix,title in zip('ab',axes,(admission,repair),
                ('Accepted / tested conditions','Repaired / nominal collision conditions')):
            image=ax.imshow(matrix,origin='lower',aspect='auto',cmap=cmap,vmin=0,vmax=1)
            ax.set_xticks(range(0,len(times),2),[f'{1000*t:g}' for t in times[::2]])
            ax.set_yticks(range(len(heights)),[f'{100*h:g}' for h in heights])
            ax.set_xlabel('Remaining time at activation (ms)')
            ax.set_ylabel('Obstacle height (cm)')
            ax.set_title(title,fontsize=7,pad=8)
            ax.text(-.10,1.13,label,transform=ax.transAxes,fontweight='bold')
            figure.colorbar(image,ax=ax,label='Condition fraction')
        figure.text(.1,.015,'Grey: untested or no nominal collision to repair.',fontsize=6)
        with (root/'data/heatmap_source.csv').open('w') as stream:
            writer=csv.writer(stream);writer.writerow(['remaining_s','height_m','conditions','admitted','nominal_collisions','repaired_collisions']);writer.writerows(source)
        with (root/'data/repair_boundary.csv').open('w') as stream:
            writer=csv.writer(stream);writer.writerow(['remaining_s','repaired_conditions','max_repaired_obstacle_height_m','max_repaired_nominal_clearance_deficit_m']);writer.writerows(limits)
        save_figure(figure,root/'figure/update_boundary',axes=list(axes),panel_ids=['a','b'],column_groups=[['a','b']])
    else:
        figure,axes=plt.subplots(1,3,figsize=(7.2,2.7))
        figure.subplots_adjust(left=.085,right=.98,bottom=.27,top=.90,wspace=.62)
        for index,method in enumerate(METHODS):
            selected=[row for row in rows if row['method']==method]
            accepted=[row for row in selected if int(row['admitted'])]
            axes[0].bar(index-.17,summary[method]['safe_sampled']/len(selected)*100,
                        color=COLORS[index],width=.3,label='Sampled safe' if index==0 else None)
            axes[0].bar(index+.17,summary[method]['rescued_frozen_collisions']/len(selected)*100,
                        facecolor='white',edgecolor=COLORS[index],width=.3,
                        label='Repaired collision' if index==0 else None)
            if accepted:
                jumps=numeric(accepted,'position_jump_m')*1000
                shifts=numeric(accepted,'deadline_shift_s')*1000
                for panel,values in [(axes[1],jumps),(axes[2],shifts)]:
                    panel.plot([index-.2,index+.2],[np.median(values)]*2,color=COLORS[index],linewidth=2)
                    panel.vlines(index,values.min(),values.max(),color=COLORS[index],linewidth=1)
        for letter,ax in zip('abc',axes):
            ax.set_xticks(range(4),['Frozen','Variable','Direct','Admit'])
            ax.tick_params(axis='x', labelsize=6)
            ax.set_ylim(bottom=0)
            ax.text(-.18,1.06,letter,transform=ax.transAxes,fontweight='bold')
        axes[0].set_ylabel('Fraction of all conditions (%)')
        axes[0].legend(frameon=False,fontsize=6,loc='upper right')
        axes[0].set_ylim(0,max(60,max(summary[m]['safe_sampled']/summary[m]['conditions']*100 for m in METHODS)*1.35))
        axes[1].set_ylabel('Reference–state position mismatch (mm)')
        axes[2].set_ylabel('Local deadline extension (ms)')
        save_figure(figure,root/'figure/method_comparison')


def plot_hulls(root):
    rows=read_csv(root/'data/metrics.csv')
    figure,axes=plt.subplots(1,2,figsize=(7.2,2.7))
    figure.subplots_adjust(left=.09,right=.98,bottom=.25,top=.90,wspace=.55)
    summary={}
    for index,name in enumerate(['Bernstein','MINVO']):
        group=[row for row in rows if row['basis']==name]
        successful=[row for row in group if int(row['feasible'])]
        summary[name]=dict(conditions=len(group),feasible=len(successful))
        axes[0].bar(index,len(successful)/len(group)*100,color=COLORS[index+2],width=.6)
        axes[0].text(index, len(successful)/len(group)*100+.35,
                     f'{len(successful)}/{len(group)}', ha='center', fontsize=7)
    pairs={}
    for row in rows:
        pairs.setdefault(row['case_id'], {})[row['basis']] = row
    common=[pair for pair in pairs.values() if all(int(pair[b]['feasible']) for b in ('Bernstein','MINVO'))]
    deltas=[(float(pair['MINVO']['peak_m'])-float(pair['Bernstein']['peak_m']))*1000 for pair in common]
    axes[1].axhline(0, color='#90979c', linewidth=.7)
    axes[1].scatter(range(1,len(deltas)+1), deltas, s=10, color=COLORS[3])
    axes[1].set_xlabel('Paired condition (both feasible)')
    axes[1].set_ylabel('MINVO − Bernstein peak (mm)')
    axes[1].set_ylim(-.4, max(deltas, default=0)+.7)
    summary['paired']=dict(both_feasible=len(common),
        bernstein_only=sum(int(p['Bernstein']['feasible']) and not int(p['MINVO']['feasible']) for p in pairs.values()),
        minvo_only=sum(int(p['MINVO']['feasible']) and not int(p['Bernstein']['feasible']) for p in pairs.values()),
        peak_delta_mm=deltas)
    axes[0].set_ylabel('Certified feasible conditions (%)')
    axes[0].set_xticks([0,1],['Bernstein','MINVO'])
    axes[0].set_ylim(0, max(summary[b]['feasible']/summary[b]['conditions']*100 for b in ('Bernstein','MINVO'))*1.25)
    for label, ax in zip('ab',axes):
        ax.text(-.15,1.06,label,transform=ax.transAxes,fontweight='bold')
    (root/'data/summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    save_figure(figure,root/'figure/basis_comparison')


def plot_multi_hulls(root):
    rows=read_csv(root/'data/multi_metrics.csv')
    counts=sorted(set(int(row['controls']) for row in rows))
    figure,axes=plt.subplots(1,2,figsize=(7.2,2.8))
    figure.subplots_adjust(left=.09,right=.98,bottom=.24,top=.90,wspace=.55)
    summary={}
    for index,count in enumerate(counts):
        group=[row for row in rows if int(row['controls'])==count]
        pairs={}
        for row in group:
            pairs.setdefault(row['case_id'],{})[row['basis']]=row
        record={}
        for offset,basis,color in [(-.17,'Bernstein',COLORS[2]),(.17,'MINVO',COLORS[3])]:
            feasible=sum(int(pair[basis]['feasible']) for pair in pairs.values())
            record[basis]=dict(feasible=feasible,conditions=len(pairs))
            axes[0].bar(index+offset,100*feasible/len(pairs),width=.30,color=color,
                        label=basis if index==0 else None)
        common=[p for p in pairs.values() if all(int(p[b]['feasible']) for b in ('Bernstein','MINVO'))]
        delta=np.array([float(p['MINVO']['control_cost'])-float(p['Bernstein']['control_cost']) for p in common])
        axes[1].scatter(index+np.linspace(-.16,.16,len(delta)),delta*1e4,s=7,color=COLORS[3],alpha=.6)
        axes[1].plot([index-.20,index+.20],[np.median(delta)*1e4]*2,color=COLORS[3],linewidth=1.5)
        record['paired']=dict(both_feasible=len(common),
            bernstein_only=sum(int(p['Bernstein']['feasible']) and not int(p['MINVO']['feasible']) for p in pairs.values()),
            minvo_only=sum(int(p['MINVO']['feasible']) and not int(p['Bernstein']['feasible']) for p in pairs.values()),
            control_cost_delta_m2=delta.tolist())
        summary[str(count)]=record
    axes[0].set_ylabel('Certified feasible conditions (%)')
    axes[0].set_ylim(0,105)
    axes[0].legend(frameon=False,fontsize=7,loc='upper left')
    axes[1].axhline(0,color=COLORS[0],linewidth=.7)
    axes[1].set_ylabel('MINVO − Bernstein control cost (cm²)')
    for letter,ax in zip('ab',axes):
        ax.set_xticks(range(len(counts)),[str(count) for count in counts])
        ax.set_xlabel('Z controls (same C2 constraints per pair)')
        ax.text(-.15,1.06,letter,transform=ax.transAxes,fontweight='bold')
    (root/'data/multi_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    save_figure(figure,root/'figure/multi_basis_comparison')


def plot_collision_repair(root):
    rows=read_csv(root/'data/scan.csv')
    traces=read_csv(root/'data/trajectories.csv')
    # A declared factorial cell: 8 cm bar, phase 25%, zero delay/state shift.
    case=next(row['case_id'] for row in rows if row['method']=='Admitted' and
              float(row['height_m'])==.08 and float(row['phase'])==.25 and
              float(row['delay_s'])==0 and float(row['state_shift_m'])==0)
    figure,axes=plt.subplots(1,2,figsize=(7.2,2.9))
    figure.subplots_adjust(left=.10,right=.98,bottom=.24,top=.88,wspace=.6)
    from matplotlib.patches import Rectangle
    axes[0].add_patch(Rectangle((.185,0),.03,.08,color='#d4d4d4'))
    for method,color in [('Frozen',COLORS[0]),('Admitted',COLORS[3])]:
        trace=[row for row in traces if row['case_id']==case and row['method']==method]
        if not trace:
            raise ValueError('Trace missing for the declared collision-repair case')
        axes[0].plot(numeric(trace,'x_m'),numeric(trace,'z_m')+.019,color=color,label=method)
    axes[0].set_xlabel('Foot centre X (m)')
    axes[0].set_ylabel('Foot centre Z (m)')
    axes[0].set_ylim(0,.17)
    axes[0].legend(frameon=False,fontsize=7)
    frozen={row['case_id']:float(row['clearance_m']) for row in rows if row['method']=='Frozen'}
    source=[]
    for method,color in [('Admitted',COLORS[3]),('Variable',COLORS[1])]:
        selected=[row for row in rows if row['method']==method]
        times=sorted(set(float(row['remaining_s']) for row in selected))
        x,y=[],[]
        for t in times:
            group=[r for r in selected if abs(float(r['remaining_s'])-t)<1e-8 and frozen[r['case_id']]< -1e-6]
            if not group:continue
            repaired=sum(int(r['admitted']) and float(r['clearance_m'])>=-1e-6 for r in group)
            x.append(t*1000);y.append(repaired/len(group)*100)
            source.append([method,t,len(group),repaired])
        axes[1].scatter(x,y,s=15,color=color,label=method)
    axes[1].set_xlabel('Remaining time at activation (ms)')
    axes[1].set_ylabel('Repaired nominal collisions (%)')
    axes[1].legend(frameon=False,fontsize=7,loc='upper left')
    axes[1].set_ylim(-3,105)
    for letter,ax in zip('ab',axes):
        ax.text(-.18,1.06,letter,transform=ax.transAxes,fontweight='bold')
    with (root/'data/repair_by_time.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['method','remaining_s','nominal_collisions','repaired']);writer.writerows(source)
    save_figure(figure,root/'figure/collision_repair')


def plot_discrete_hulls(root):
    """Count violations retained by sparse constraints using an independent grid."""
    rows=read_csv(root/'data/discrete_metrics.csv')
    methods=('Sample3','Sample5','Sample9','MINVO')
    summary={}
    figure,axes=plt.subplots(1,3,figsize=(7.2,2.9))
    figure.subplots_adjust(left=.08,right=.98,bottom=.28,top=.82,wspace=.68)
    for index,controls in enumerate((11,21,41)):
        values=[]
        for method in methods:
            selected=[r for r in rows if int(r['controls'])==controls and r['basis']==method]
            if len(selected)!=180:
                raise RuntimeError('Expected all 180 conditions per decision space and method')
            accepted=[r for r in selected if int(r['feasible'])]
            row=dict(conditions=len(selected),feasible=len(accepted),
                     dense_planning_violations=sum(float(r['dense_planning_violation_m'])>1e-6 for r in accepted),
                     sampled_sphere_collisions=sum(float(r['minimum_clearance_m'])< -1e-6 for r in accepted))
            summary[f'{controls}_{method}']=row
            values.append(row)
        for ax,key in zip(axes,('feasible','dense_planning_violations','sampled_sphere_collisions')):
            ax.bar(np.arange(4)+(index-1)*.23,[r[key] for r in values],width=.21,color=COLORS[index+1],
                   label=f'{controls} controls')
    for letter,ax,label in zip('abc',axes,('Feasible QPs / 180 conditions','Accepted with a planning violation','Accepted with a sphere collision')):
        ax.set_xticks(range(4),['3','5','9','MINVO'])
        ax.set_xlabel('Point constraints / continuous hull')
        ax.set_ylabel(label)
        ax.set_ylim(0,max(1,max(patch.get_height() for patch in ax.patches))*1.12)
        ax.text(-.18,1.08,letter,transform=ax.transAxes,fontweight='bold')
    axes[0].legend(frameon=False,fontsize=6,loc='lower left',bbox_to_anchor=(0,1.13),ncol=3)
    (root/'data/discrete_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    save_figure(figure,root/'figure/discrete_vs_continuous')
