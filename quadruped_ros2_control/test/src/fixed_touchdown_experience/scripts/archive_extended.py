#!/usr/bin/env python3
"""Archive code/config provenance and hash raw data and figure artifacts."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data',type=Path,required=True);p.add_argument('--figure',type=Path,required=True);a=p.parse_args()
    ws=next(v for v in Path(__file__).resolve().parents if (v/'src/quadruped_ros2_control/controllers').is_dir())
    pkg=ws/'src/quadruped_ros2_control/test/src/fixed_touchdown_experience';out=a.data/'provenance';out.mkdir(exist_ok=True)
    pilot=json.loads((a.data/'gazebo_pilot.manifest.json').read_text())
    sources=[ws/k for k in pilot if not k.startswith('trial.') and (ws/k).is_file() and not k.startswith(('build/','install/'))]
    sources += [f for directory in [pkg/'scripts',pkg/'src'] for f in directory.glob('*') if f.is_file() and f.suffix in ['.py','.cpp']]
    sources += [pkg/name for name in ['CMakeLists.txt','package.xml','EXTENDED_EXPERIMENTS.md']]
    sources += [ws/name for name in ['论文初稿.md','论文需要讲清楚的东西.md','待完善.md'] if (ws/name).exists()]
    sources += list((ws/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller').glob('include/**/*.h'))
    sources += list((ws/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller').glob('src/**/*.cpp'))
    records={}
    for source in sorted(set(sources)):
        rel=source.relative_to(ws);target=out/'sources'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target);records[str(rel)]=sha(source)
    binary={}
    for f in [ws/'build/fixed_touchdown_experience/feasibility_sweep',ws/'build/fixed_touchdown_experience/hull_export',ws/'install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so']:
        binary[str(f.relative_to(ws))]=sha(f)
    raw={str(f.relative_to(a.data)):sha(f) for f in a.data.glob('*') if f.is_file()}
    raw.update({str(f.relative_to(a.data)):sha(f) for f in (a.data/'sweep').glob('*') if f.is_file()})
    figures={f.name:sha(f) for f in a.figure.glob('*') if f.is_file()}
    cpu=next((s.split(':',1)[1].strip() for s in Path('/proc/cpuinfo').read_text().splitlines() if s.startswith('model name')),'unknown')
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ws/'src',text=True).strip()
    (out/'manifest.json').write_text(json.dumps(dict(git_head=revision,working_tree_modified=True,
        sources=records,binaries=binary,data=raw,figures=figures,cpu=cpu,platform=platform.platform(),python=sys.version,
        evidence='One Gazebo pilot plus local conditional production replay; not 6400 physical trials',
        source_world_preserved=True,offline_workspace_gate=False,offline_wbc_gate=False),indent=2,ensure_ascii=False)+'\n')
    (out/'tracked_changes.patch').write_bytes(subprocess.check_output(['git','diff','--binary'],cwd=ws/'src'))
    print('Provenance:',out)

if __name__=='__main__':main()
