#!/usr/bin/env python3
"""Launch an isolated Gazebo trial and stop only processes belonging to this trial.
Run in a sourced ROS workspace. JSON/CSV, configuration snapshots and a Gazebo log
are retained beside --output. Exit success only if scenario_probe passes.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', required=True, choices=['stairs', 'platform'])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--domain', type=int, default=82)
    p.add_argument('--x', type=float, default=0.)
    p.add_argument('--y', type=float, default=0.)
    p.add_argument('--speed', type=float, default=.5)
    p.add_argument('--sim-seconds', type=float, default=100.)
    args = p.parse_args()
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), GZ_PARTITION='terrain_trial_'+str(args.domain))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    workspace = next(parent for parent in Path(__file__).resolve().parents
                     if (parent/'src/quadruped_ros2_control/descriptions').is_dir())
    config = workspace/'src/quadruped_ros2_control/descriptions/cyberdog_description/config/ocs2'
    for name in ['task.info', 'gait.info', 'reference.info']:
        args.output.with_suffix('.'+name).write_text((config/name).read_text())
    for source in [config.parent/'gazebo.yaml', config.parent.parent/'xacro/gazebo.xacro', workspace/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/config/elevation_mapping.yaml']:
        args.output.with_suffix('.'+source.name).write_text(source.read_text())
    sources = [config/'task.info', config/'gait.info', config/'reference.info',
               config.parent.parent/'xacro/gazebo.xacro', Path(__file__),
               Path(__file__).with_name('scenario_probe.py')]
    controller = workspace/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller'
    sources += [controller/'src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp',
                controller/'src/interface/constraint/SwingTrajectoryQPOptimizer.cpp',
                controller/'src/interface/constraint/TerrainSafetyCorridor.cpp',
                controller/'src/wbc/WeightedWbc.cpp']
    manifest = {str(path.relative_to(workspace)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sources}
    args.output.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    logpath = args.output.with_suffix('.launch.log')
    with logpath.open('w') as log:
        launch = subprocess.Popen(['ros2','launch','ocs2_quadruped_controller','gazebo.launch.py',
                                   'world:=stair10cm',f'x:={args.x}',f'y:={args.y}','gui:=false','rviz:=false'],
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+60
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError('Gazebo exited; see '+str(logpath))
                if 'Configured and activated' in logpath.read_text(errors='replace'):
                    break
                time.sleep(.2)
            else:
                raise RuntimeError('Controller activation timeout; see '+str(logpath))
            return subprocess.call([sys.executable, str(Path(__file__).with_name('scenario_probe.py')),
                                    '--scenario', args.scenario, '--output', str(args.output),
                                    '--speed', str(args.speed), '--sim-seconds', str(args.sim_seconds)], env=env)
        finally:
            # The process group was created here; unrelated user simulations are untouched.
            for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
                try:
                    os.killpg(launch.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    launch.wait(timeout=6)
                    break
                except subprocess.TimeoutExpired:
                    pass


if __name__ == '__main__':
    raise SystemExit(main())
