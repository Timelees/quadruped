#!/usr/bin/env python3
"""Check observed bilinear-patch prism heights against static SDF box bounds.

Four finite 2 cm corner observations form one known interpolation patch, exactly
as GridMapTerrainInterface. Rotated boxes use a conservative AABB truth bound.
No credit is taken for horizontal dilation. Unknown and missing corners remain
uncertified, and samples from different timestamps are never fused.
"""
import csv
import json
from pathlib import Path
import re

import numpy as np
from analyze_hocbf_trials import quantiles
from analyze_true_geometry import world_boxes


def analyze(path):
    manifest = json.loads(path.with_suffix('.manifest.json').read_text())
    if 'trial.map_sampling' not in manifest:
        return dict(available=False, reason='Full-resolution ROI samples unavailable')
    if not path.with_suffix('.world.sdf').exists():
        return dict(available=False, reason='No frozen SDF for this early diagnostic launch')
    budget = float(re.search(r'^\s*corridorHeightError\s+(\S+)', path.with_suffix('.task.info').read_text(), re.M)[1])
    r = manifest['trial.map_sampling']['near_obstacles_m']
    centres = [bar[0] for bar in manifest.get('trial.controlled_bars',[(1.1,),(1.8,),(2.5,),(3.2,)])]
    boxes=[]
    for centre,rotation,half in world_boxes(path.with_suffix('.world.sdf')):
        extent=np.abs(rotation)@half
        boxes.append((centre-extent, centre+extent))
    frames={}
    with path.with_suffix('.certified_map.csv').open() as stream:
        for row in csv.DictReader(stream):
            x,y,h=map(float,(row['x'],row['y'],row['certified_height_m']))
            frames.setdefault(row['ros_time'],{})[(round(x,7),round(y,7))]=h
    excess=[]; missing=unknown=0
    for points in frames.values():
        for (x,y),height in points.items():
            if not (abs(y+3)<.8 and any(abs(x-c)<.12 for c in centres)): continue
            keys=[(round(x+dx*r,7),round(y+dy*r,7)) for dx in (0,1) for dy in (0,1)]
            if not all(k in points for k in keys): missing+=1;continue
            heights=np.array([points[k] for k in keys])
            if not np.all(np.isfinite(heights)): unknown+=1;continue
            true_upper=0.
            for lo,hi in boxes:
                if lo[0]<=x+r and hi[0]>=x and lo[1]<=y+r and hi[1]>=y:
                    true_upper=max(true_upper,float(hi[2]))
            excess.append(true_upper-float(heights.max())-budget)
    return dict(available=True,height_budget_m=budget,known_patches=len(excess),unknown_patches=unknown,
                missing_corner_patches=missing,true_minus_prism_minus_budget_m=quantiles(excess),
                violating_patch_samples=sum(e>1e-6 for e in excess),
                definition='Full observed 2cm interpolation patches near four bars, each map separately; static SDF AABB upper; no horizontal dilation credit; finite sampled ROI only')
