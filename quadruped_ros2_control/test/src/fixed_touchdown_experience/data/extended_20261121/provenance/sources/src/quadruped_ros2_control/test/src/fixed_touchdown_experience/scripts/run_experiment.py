#!/usr/bin/env python3
"""Reproduce the single-bar route. Run after sourcing ROS and this workspace."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def workspace():
    return next(p for p in Path(__file__).resolve().parents
                if (p/'src/quadruped_ros2_control/controllers').is_dir())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True, help='New run stem; existing data cannot be overwritten')
    p.add_argument('--domain', type=int, default=171)
    p.add_argument('--height', type=float, default=.10)
    p.add_argument('--speed', type=float, default=.4)
    p.add_argument('--seconds', type=float, default=65.)
    p.add_argument('--reveal', action='store_true', help='Controlled late certificate reveal; not perception validation')
    p.add_argument('--reveal-phase', type=float, default=.10)
    p.add_argument('--reveal-x', type=float, default=-.65)
    p.add_argument('--truth-map', action='store_true', help='Labelled world-geometry oracle intervention')
    p.add_argument('--safety', choices=['hocbf','full','off'], default='hocbf')
    p.add_argument('--transport-delay', type=float)
    p.add_argument('--vertical-extent', type=float)
    p.add_argument('--seed', type=int, default=42)
    a=p.parse_args()
    if not .10 <= a.height <= .13: p.error('Use the requested 10–13 cm obstacle range')
    if a.truth_map and not a.reveal: p.error('--truth-map requires --reveal')
    root=workspace(); source=root/'src/quadruped_ros2_control/test/src/terrainSafetyCorridor_test/run_trial.py'
    cmd=[sys.executable,str(source),'--scenario','obstacle','--domain',str(a.domain),
         '--output',str(a.output.resolve()),'--obstacle-height',str(a.height),'--speed',str(a.speed),
         '--sim-seconds',str(a.seconds),'--safety-filter',a.safety,'--online-replan','on','--seed',str(a.seed)]
    if a.reveal:
        cmd+=['--controlled-reveal','--reveal-at-x',str(a.reveal_x),'--reveal-phase',str(a.reveal_phase),
              '--reveal-require-crossing','--controlled-map-source','world-truth' if a.truth_map else 'perception']
    if a.transport_delay is not None:cmd+=['--mrt-transport-delay',str(a.transport_delay)]
    if a.vertical_extent is not None:cmd+=['--corridor-vertical-extent',str(a.vertical_extent)]
    a.output.parent.mkdir(parents=True,exist_ok=True)
    protocol=a.output.with_suffix('.protocol.json')
    if protocol.exists() or a.output.with_suffix('.json').exists():p.error('Run already exists')
    protocol.write_text(json.dumps(dict(argv=cmd,route='spawn yaw 0; turn 180 deg; forward only',
        evidence='Gazebo',hocbf=a.safety!='off',controlled_reveal=a.reveal,
        sensor_claim='truth-map intervention' if a.truth_map else 'perception'),indent=2)+'\n')
    return subprocess.call(cmd,env=dict(os.environ),cwd=root)

if __name__=='__main__':sys.exit(main())
