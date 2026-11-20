#!/usr/bin/env python3
"""Plot independent acceptance launches, preserving failures and missing evidence.

Figure contract: distinguish mechanisms that execute from physical assumptions
that remain falsified. Quantitative grids, 183 mm width, >=7 pt editable text.
Each point is one independent launch; no significance tests or pooled success
claim. Summary quantiles describe time samples, not independent replicates.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--audit-tools', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.audit_tools))
    from audit_panel_alignment import require_matplotlib_panel_alignment
    records = json.loads(args.summary.read_text())['trials']
    args.output.mkdir(exist_ok=True, parents=True)
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],
        'font.size':7,'axes.titlesize':8,'axes.labelsize':7,'xtick.labelsize':7,'ytick.labelsize':7,
        'legend.fontsize':7,'pdf.fonttype':42,'svg.fonttype':'none',
        'axes.spines.right':False,'axes.spines.top':False})
    colors = {'off':'#7b858e','full':'#287b89','frozen':'#7b858e','direct':'#b66c36','admitted':'#287b89'}
    source=[]

    def pair():
        fig, axes=plt.subplots(1,2,figsize=(7.2,2.8),layout='constrained')
        for label,ax in zip(('a','b'),axes):
            ax.text(-.17,1.08,label,transform=ax.transAxes,fontweight='bold',fontsize=9)
        return fig,axes

    def save(fig,name):
        fig.canvas.draw()
        require_matplotlib_panel_alignment(fig,json_out=args.output/(name+'.alignment.json'),strict=True)
        fig.savefig(args.output/(name+'.pdf'))
        fig.savefig(args.output/(name+'.svg'))
        fig.savefig(args.output/(name+'.png'),dpi=600)
        plt.close(fig)

    stable=[r for r in records if r['trial'].startswith(('release_','remaining_')) and r['scenario'] in ('flat','platform')]
    if stable:
        fig,axes=pair()
        labels=['Flat\nOff','Flat\nFull','Platform\nOff','Platform\nFull']
        for r in stable:
            if 'hold' not in r['stability']: continue
            x=2*int(r['scenario']=='platform')+int(r['safety_filter']=='full')
            x+=(int(r['seed'])%2-.5)*.16
            s=r['stability']['hold']
            jitter=r['stability']['hold_first_5s']
            values=[100*s['drift_from_first_m'],np.linalg.norm(jitter['angular_rate_rms_rps'])]
            for ax,value in zip(axes,values):
                ax.scatter(x,value,s=24,marker='o' if r['result']=='passed' else 'x',color=colors[r['safety_filter']])
            source.append(dict(figure='stability',trial=r['trial'],result=r['result'],x=x,
                drift_cm=values[0],angular_rate_rms=values[1],hold_duration=s['duration_s'],
                jitter_window_observed_s=jitter['duration_s']))
        for ax in axes: ax.set_xticks(range(4),labels);ax.set_xlim(-.45,3.45)
        axes[0].set(ylabel='Hold displacement (cm)',title='Endpoint hold; each point = one launch')
        axes[1].set(ylabel='Angular velocity RMS (rad/s)',title='First 5 s of hold; all observed samples')
        axes[1].set_ylim(bottom=0)
        save(fig,'stability')

    delays=[r for r in records if r['trial'].startswith(('delay_','remaining_delay_'))]
    if delays:
        fig,axes=pair()
        requested=sorted({1000*r['transport_delay_s'] for r in delays})
        for r in delays:
            x=1000*r['transport_delay_s']
            q=r['injected_transport_residence_s']
            if not q['n']: continue
            median,p95,_=1000*np.array(q['P50_P95_P99'])
            marker='o' if r['result']=='passed' else 'x'
            axes[0].plot([x,x],[median,p95],color=colors['full'],lw=1)
            axes[0].scatter(x,median,s=25,marker=marker,color=colors['full'])
            axes[1].scatter(x,r['replan_applied'],s=25,marker=marker,color=colors['full'])
            source.append(dict(figure='delay',trial=r['trial'],result=r['result'],requested_ms=x,
                median_ms=median,p95_ms=p95,applied=r['replan_applied']))
        axes[0].plot([0,max(requested)],[0,max(requested)],ls=':',color='.5',lw=.8)
        axes[0].set(ylabel='Measured residence (ms)',title='Median and P95 after injection starts')
        axes[1].set(ylabel='New references actually used (count)',title='Circle: passed; cross: acceptance not met')
        for ax in axes: ax.set(xlabel='Requested MRT hold (ms)');ax.set_xticks(requested)
        save(fig,'transport_delay')

    reveal=[r for r in records if r['trial'].startswith('reveal_crossing_')]
    if reveal:
        fig,axes=pair()
        methods=['frozen','direct','admitted']
        counts=[]
        for r in reveal:
            window=r.get('reveal_swing_window')
            if not window or not window['true_LF_spanned_bar']:
                source.append(dict(figure='reveal',trial=r['trial'],result=r['result'],
                    trigger_present=bool(window),true_LF_spanned_bar=False))
                continue
            x=methods.index(r['method'])+(int(r['seed'])%2-.5)*.16
            count=len(window['direct']) if r['method']=='direct' else len(window['applied'])
            counts.append(count)
            jumps=[1000*v['reference_jump_m'] for v in window['direct']]
            axes[0].scatter(x,count,s=25,color=colors[r['method']],marker='o' if r['result']=='passed' else 'x')
            if jumps: axes[1].scatter(x,max(jumps),s=25,color=colors[r['method']],marker='o' if r['result']=='passed' else 'x')
            source.append(dict(figure='reveal',trial=r['trial'],result=r['result'],applied_LF_in_revealed_swing=count,
                trigger_present=True,true_LF_spanned_bar=True,direct_jump_max_mm=max(jumps) if jumps else None))
        for ax in axes: ax.set_xticks(range(3),['Frozen','Direct','Admitted']);ax.set_xlim(-.45,2.45)
        axes[0].set_yticks(range(max(counts,default=0)+1))
        axes[0].set_ylim(-.15,max(1,max(counts,default=0))+.25)
        axes[0].set(ylabel='LF updates actually used (count)',title='True crossing + reveal required')
        axes[1].set(ylabel='Direct reference jump (mm)',title='Delivered direct updates only')
        axes[1].set_ylim(bottom=0)
        save(fig,'controlled_update')

    maps=[r for r in records if r['trial'].startswith('map_budget_')]
    if maps:
        fig,axes=pair()
        for r in maps:
            m=r['map_prism_envelope'];x=1000*m['height_budget_m'];q=m['true_minus_prism_minus_budget_m']
            if q['n']:
                med,p95,_=1000*np.array(q['P50_P95_P99']);maximum=1000*q['maximum']
                axes[0].plot([x,x],[med,maximum],color=colors['full'],lw=1)
                axes[0].scatter(x,p95,color=colors['full'],s=25)
            total=m['known_patches']+m['unknown_patches']
            if total: axes[1].scatter(x,m['unknown_patches']/total,color=colors['full'],s=25)
            source.append(dict(figure='map',trial=r['trial'],result=r['result'],height_budget_mm=x,
                known_patches=m['known_patches'],unknown_patches=m['unknown_patches'],violations=m['violating_patch_samples']))
        axes[0].axhline(0,color='.5',lw=.8,ls=':')
        axes[0].set(ylabel='Truth − prism − budget (mm)',title='P95 point; median-to-maximum line')
        axes[1].set(ylabel='Unknown patch fraction',ylim=(0,1),title='Unknowns retain no certificate')
        for ax in axes:ax.set(xlabel='Configured height budget (mm)');ax.set_xticks([0,5,10])
        save(fig,'map_envelope')

    physical=[r for r in records if r['trial'].startswith(('release_','remaining_')) and r['safety_filter']=='full']
    if physical:
        fig,axes=pair()
        shown=[]
        for r in physical:
            q=r['observed_contact_free_interior_disturbance_lower_bound_mps2']
            if not q['n']: continue
            x=len(shown);shown.append(r['trial'].replace('release_','').replace('remaining_','').replace('_full',''))
            median,p95,_=q['P50_P95_P99']
            axes[0].plot([x,x],[median,q['maximum']],color=colors['full'])
            axes[0].scatter(x,p95,s=25,color=colors['full'])
            tq=r['next_sample_torque_mismatch_Nm']
            if tq['n']:axes[1].scatter(x,tq['P50_P95_P99'][1],s=25,color=colors['full'])
            source.append(dict(figure='physical',trial=r['trial'],result=r['result'],residual_median=median,
                residual_p95=p95,residual_max=q['maximum'],samples=q['n']))
        axes[0].axhline(.5,color=colors['direct'],ls='--',lw=.9)
        axes[0].set(ylabel='Residual bound under assumed errors (m/s²)',title='P95 point; median-to-maximum line')
        axes[1].set(ylabel='Next-sample torque mismatch P95 (Nm)',title='Predicted torque versus readback')
        axes[1].set_ylim(bottom=0)
        for ax in axes:ax.set_xticks(range(len(shown)),[s.replace('_','\n') for s in shown])
        save(fig,'execution_budget')

    fields=sorted({key for row in source for key in row})
    with (args.output/'source_data.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fields);writer.writeheader();writer.writerows(source)


if __name__=='__main__': main()
