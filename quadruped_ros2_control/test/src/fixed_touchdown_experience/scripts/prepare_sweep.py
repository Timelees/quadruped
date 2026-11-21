#!/usr/bin/env python3
"""Freeze the eight-height scene without changing the user's source SDF."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    tree=ET.parse(a.world);records=[]
    for cm in range(2,17,2):
        model=tree.getroot().find(f".//model[@name='thin_obstacle_{cm}cm']")
        if model is None:raise ValueError(f'Missing {cm} cm bar')
        size=list(map(float,model.find('.//collision/geometry/box/size').text.split()))
        pose=list(map(float,model.find('pose').text.split()))
        assert abs(size[2]-cm/100)<1e-9 and len(pose)==6 and pose[3:]==[0,0,0]
        records.append(dict(name=model.get('name'),source_pose=pose.copy(),size=size,
                            source_bottom=pose[2]-size[2]/2,source_top=pose[2]+size[2]/2))
        pose[2]=size[2]/2;model.find('pose').text=' '.join(map(str,pose))
        records[-1]['experiment_pose']=pose;records[-1]['experiment_top']=size[2]
    corrected=a.output/'eight_heights.sdf';tree.write(corrected,encoding='unicode')
    # The existing single-route collector uses this first bar for its manifest;
    # all eight physical obstacles remain present in the pilot scene.
    tree.getroot().find(".//model[@name='thin_obstacle_2cm']").set('name','thin_obstacle')
    tree.write(a.output/'pilot.sdf',encoding='unicode')
    (a.output/'scene.json').write_text(json.dumps(dict(source=str(a.world.resolve()),
        source_sha256=hashlib.sha256(a.world.read_bytes()).hexdigest(),obstacles=records,
        corrected_sha256=hashlib.sha256(corrected.read_bytes()).hexdigest()),indent=2)+'\n')
    print(a.output)

if __name__=='__main__':main()
