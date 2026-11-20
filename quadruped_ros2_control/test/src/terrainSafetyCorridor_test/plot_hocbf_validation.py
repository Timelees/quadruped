#!/usr/bin/env python3
"""Plot every declared validation run, including failures and geometry-only mode.

Success intervals use independent simulator launches. Diagnostic coverage and
residuals are conditional sample statistics, never independent repetitions.
"""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

COLORS={'off':'#777777','hocbf':'#b66d36','full':'#287b8e'}
LABELS={'off':'No filter','hocbf':'HOCBF','full':'Full + degradation'}


def save(fig,stem):
    from audit_panel_alignment import require_matplotlib_panel_alignment
    require_matplotlib_panel_alignment(fig,json_out=str(stem)+'.alignment.json',strict=True)
    fig.savefig(str(stem)+'.pdf')
    fig.savefig(str(stem)+'.svg')
    fig.savefig(str(stem)+'.png',dpi=600)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--audit-tools',type=Path,required=True)
    args=parser.parse_args();sys.path.insert(0,str(args.audit_tools))
    from analyze_hocbf_trials import wilson
    data=json.loads(args.summary.read_text())['trials']
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],
        'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False})
    args.output.mkdir(parents=True,exist_ok=True)
    scenarios=list(dict.fromkeys(d['scenario'] for d in data))
    fig,axes=plt.subplots(1,len(scenarios),figsize=(7.2,3.1),squeeze=False,layout='constrained',sharey=True)
    for letter,scenario,ax in zip('abcdefghijklmnopqrstuvwxyz',scenarios,axes[0]):
        for index,mode in enumerate(('off','hocbf','full')):
            selected=[d for d in data if d['scenario']==scenario and d['mode']==mode]
            if not selected:continue
            n=len(selected);k=sum(d['result']=='passed' for d in selected);ci=wilson(k,n);p=k/n
            ax.errorbar(index,p,yerr=np.array([[p-ci[0]],[ci[1]-p]]),fmt='o',color=COLORS[mode],capsize=4,ms=6,lw=1.2)
            ax.annotate(f'{k}/{n}',(index,p),xytext=(8,0),textcoords='offset points',va='center',fontsize=8)
        ax.set(xticks=[0,1,2],xticklabels=['Off','HOCBF','Full'],xlim=(-.4,2.6),ylim=(-.08,1.12),
            title='12 cm obstacle' if scenario=='obstacle' else 'Four bars + pulse')
        ax.annotate(letter,(0,1),xycoords='axes fraction',xytext=(-14,8),textcoords='offset points',weight='bold')
    axes[0,0].set_ylabel('Traversal success fraction')
    save(fig,args.output/'validation_success')
    # One dot per independent launch; Full's line links geometry and tube coverage.
    fig,axes=plt.subplots(1,len(scenarios),figsize=(7.2,3.1),squeeze=False,layout='constrained',sharey=True)
    for letter,scenario,ax in zip('abcdefghijklmnopqrstuvwxyz',scenarios,axes[0]):
        subset=[d for d in data if d['scenario']==scenario]
        seeds=sorted({d['seed'] for d in subset})
        offsets=np.linspace(-.12,.12,len(seeds))
        for seed,offset in zip(seeds,offsets):
            selected={d['mode']:d for d in subset if d['seed']==seed}
            for index,mode in enumerate(('off','hocbf','full')):
                if mode not in selected:continue
                d=selected[mode];g=100*d['geometry_fraction'];c=100*d['contract_fraction']
                ax.plot(index+offset,g,'o',color=COLORS[mode],ms=5)
                if mode=='full':
                    ax.plot([index+offset,index+offset],[c,g],color=COLORS[mode],lw=1)
                    ax.plot(index+offset,c,'s',mfc='white',mec=COLORS[mode],ms=4)
        ax.set(xticks=[0,1,2],xticklabels=['Off','HOCBF','Full'],xlim=(-.4,2.4),ylim=(-2,102),
            title='12 cm obstacle' if scenario=='obstacle' else 'Four bars + pulse')
        ax.annotate(letter,(0,1),xycoords='axes fraction',xytext=(-14,8),textcoords='offset points',weight='bold')
    axes[0,0].plot([], [], 'o', color='.4', label='Geometry')
    axes[0,0].plot([], [], 's', mfc='white', mec='.4', label='Geometry + tube')
    axes[0,0].legend(frameon=False,loc='upper left',fontsize=8)
    axes[0,0].set_ylabel('Geometry / all swing samples (%)')
    save(fig,args.output/'validation_coverage')
    # Paired binary outcomes expose gains, losses and ties without pooling samples.
    paired={}
    for scenario in scenarios:
        paired[scenario]={}
        for mode in ('hocbf','full'):
            baseline={d['seed']:d for d in data if d['scenario']==scenario and d['mode']=='off'}
            selected={d['seed']:d for d in data if d['scenario']==scenario and d['mode']==mode}
            pairs=[(baseline[k]['result']=='passed',selected[k]['result']=='passed') for k in baseline.keys()&selected.keys()]
            win=sum(not a and b for a,b in pairs);loss=sum(a and not b for a,b in pairs)
            paired[scenario][mode]=dict(pairs=len(pairs),wins=win,losses=loss,ties=len(pairs)-win-loss)
    (args.output/'validation_pairs.json').write_text(json.dumps(paired,indent=2)+'\n')


if __name__=='__main__':
    main()
