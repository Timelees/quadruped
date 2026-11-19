#!/usr/bin/env python3
"""Launch an isolated Gazebo trial and stop only processes belonging to this trial.
Run in a sourced ROS workspace. JSON/CSV, configuration snapshots and a Gazebo log
are retained beside --output. Exit success only if scenario_probe passes.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import re
import xml.etree.ElementTree as ET
import subprocess
import sys
import time


def trial_processes(partition):
    """Find this launch's children even if ros2 launch has already exited."""
    marker = ('GZ_PARTITION=' + partition).encode()
    pids = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if marker in (entry/'environ').read_bytes().split(b'\0'):
                pids.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return pids


def stop_trial(launch, partition):
    # ROS launch gives subprocesses their own process groups. Waiting only for
    # the launch parent can leave a Gazebo server running and exhausting RAM.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pid in trial_processes(partition):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
        deadline = time.monotonic() + 6
        while trial_processes(partition) and time.monotonic() < deadline:
            launch.poll()
            time.sleep(.2)
        if not trial_processes(partition):
            break
    launch.wait(timeout=2)


def available_memory_bytes():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) * 1024
    raise RuntimeError('Cannot determine available memory before launching Gazebo')


def override_task(env, source, output, settings):
    """Expose an isolated robot description via the ament index."""
    if not settings:
        return source/'config/ocs2'
    overlay = output.resolve().with_suffix('.profile')
    overlay.mkdir()
    package = source.name
    resource = overlay/'share/ament_index/resource_index/packages'/package
    resource.parent.mkdir(parents=True)
    resource.touch()
    share = overlay/'share'/package
    share.mkdir()
    for child in source.iterdir():
        if child.name != 'config':
            (share/child.name).symlink_to(child, target_is_directory=child.is_dir())
    shutil.copytree(source/'config', share/'config')
    config = share/'config/ocs2'
    task = config/'task.info'
    contents = task.read_text()
    for key, value in settings.items():
        contents, n = re.subn(r'^(\s*'+key+r'\s+)(true|false|[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\b', r'\g<1>'+value,
                              contents, flags=re.M)
        if n == 0 and key == 'safetyFilterTubeEnabled':
            contents = re.sub(r'^(\s*safetyFilterEnabled\s+(?:true|false)[^\n]*)$',
                              r'\1\n  safetyFilterTubeEnabled '+value, contents, flags=re.M)
        elif n != 1:
            raise ValueError(f'Expected one task.info setting for {key}, found {n}')
    task.write_text(contents)
    env['AMENT_PREFIX_PATH'] = str(overlay)+os.pathsep+env.get('AMENT_PREFIX_PATH', '')
    return config

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', required=True, choices=['trot', 'stairs', 'platform', 'obstacle', 'trench', 'piles'])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--domain', type=int, default=82)
    p.add_argument('--x', type=float, default=None)
    p.add_argument('--y', type=float, default=0.0)
    p.add_argument('--yaw', type=float, default=0.0, help='Spawn heading in radians')
    p.add_argument('--speed', type=float, default=.5)
    p.add_argument('--sim-seconds', type=float, default=100.)
    p.add_argument('--obstacle-height', type=float, help='Obstacle top height in metres; writes an isolated world copy')
    p.add_argument('--safety-filter', choices=['off', 'hocbf', 'full'])
    p.add_argument('--online-replan', choices=['on', 'off'])
    p.add_argument('--safety-max-correction', type=float, help='Per-axis safety acceleration correction budget [m/s²]')
    p.add_argument('--safety-contact-force-deviation', type=float,
                   help='Per-component contact force deviation from baseline WBC [N]')
    p.add_argument('--safety-tube-rate', type=float, help='Tube barrier rate for an isolated tuning trial')
    p.add_argument('--safety-extra-sample-rate', type=float, help='Extra tube sample reserve beyond exact hold constraints')
    p.add_argument('--safety-disturbance-bound', type=float, help='Empirical acceleration disturbance budget [m/s²]')
    p.add_argument('--safety-hocbf-gain', type=float, help='Set both HOCBF gains in an isolated calibration trial')
    p.add_argument('--wrench-force', type=float, default=0., help='Downward force on FL knee [N], isolated Gazebo wrench plugin')
    p.add_argument('--wrench-duration', type=float, default=.06, help='Requested pulse duration in simulation seconds')
    args = p.parse_args()
    args.output = args.output.with_suffix('.json')
    if not 0 <= args.domain <= 232:
        p.error('--domain must be in [0,232] for the default Fast DDS port mapping')
    # Keep headroom for the desktop; do not wait until swap is exhausted.
    memory_reserve = 1024 * 1024 * 1024
    if available_memory_bytes() < memory_reserve:
        p.error('Less than 1 GiB memory available; Gazebo was not started')
    if args.x is None:
        args.x = 0.0  # gazebo.launch.py default; reverse-side trials use --x -2
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), GZ_PARTITION='terrain_trial_'+str(args.domain))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    workspace = next(parent for parent in Path(__file__).resolve().parents
                     if (parent/'src/quadruped_ros2_control/descriptions').is_dir())
    description = workspace/'src/quadruped_ros2_control/descriptions/cyberdog_description'
    settings = {}
    if args.safety_filter is not None:
        settings['safetyFilterEnabled'] = 'false' if args.safety_filter == 'off' else 'true'
        settings['safetyFilterTubeEnabled'] = 'true' if args.safety_filter == 'full' else 'false'
    if args.online_replan is not None:
        settings['swingOnlineReplanEnabled'] = 'true' if args.online_replan == 'on' else 'false'
    if args.safety_max_correction is not None:
        if not math.isfinite(args.safety_max_correction) or args.safety_max_correction <= 0:
            p.error('--safety-max-correction must be positive and finite')
        settings['safetyFilterMaxCorrection'] = str(args.safety_max_correction)
    if args.safety_contact_force_deviation is not None:
        if not math.isfinite(args.safety_contact_force_deviation) or args.safety_contact_force_deviation <= 0:
            p.error('--safety-contact-force-deviation must be positive and finite')
        settings['safetyContactForceDeviation'] = str(args.safety_contact_force_deviation)
    if args.safety_tube_rate is not None:
        if not math.isfinite(args.safety_tube_rate) or args.safety_tube_rate <= 0:
            p.error('--safety-tube-rate must be positive and finite')
        settings['safetyFilterTubeRate'] = str(args.safety_tube_rate)
    if args.safety_extra_sample_rate is not None:
        if not math.isfinite(args.safety_extra_sample_rate) or args.safety_extra_sample_rate < 0:
            p.error('--safety-extra-sample-rate must be nonnegative and finite')
        settings['safetyFilterTubeSampleRate'] = str(args.safety_extra_sample_rate)
    if args.safety_disturbance_bound is not None:
        if not math.isfinite(args.safety_disturbance_bound) or args.safety_disturbance_bound < 0:
            p.error('--safety-disturbance-bound must be nonnegative and finite')
        settings['safetyFilterDisturbanceBound'] = str(args.safety_disturbance_bound)
    if args.safety_hocbf_gain is not None:
        if not math.isfinite(args.safety_hocbf_gain) or args.safety_hocbf_gain <= 0:
            p.error('--safety-hocbf-gain must be positive and finite')
        for key in ('safetyFilterHocbfGain1', 'safetyFilterHocbfGain2'):
            settings[key] = str(args.safety_hocbf_gain)
    config = override_task(env, description, args.output, settings)
    for name in ['task.info', 'gait.info', 'reference.info']:
        args.output.with_suffix('.'+name).write_text((config/name).read_text())
    for source in [config.parent/'gazebo.yaml', config.parent.parent/'xacro/gazebo.xacro', workspace/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller/config/elevation_mapping.yaml']:
        args.output.with_suffix('.'+source.name).write_text(source.read_text())
    sources = [description/'config/ocs2'/name for name in ['task.info', 'gait.info', 'reference.info']]
    sources += [description/'xacro/gazebo.xacro', Path(__file__),
               Path(__file__).with_name('scenario_probe.py')]
    controller = workspace/'src/quadruped_ros2_control/controllers/ocs2_quadruped_controller'
    sources += [controller/'src/perceptive/interface/PerceptiveLeggedReferenceManager.cpp',
                controller/'src/interface/constraint/SwingTrajectoryQPOptimizer.cpp',
                controller/'src/interface/constraint/TerrainSafetyCorridor.cpp',
                controller/'src/wbc/WeightedWbc.cpp', controller/'src/wbc/WbcBase.cpp',
                controller/'src/FSM/StateOCS2.cpp', controller/'src/interface/constraint/SwingTrajectoryPlanner.cpp',
                controller/'src/interface/constraint/SwingOnlineReplanner.cpp',
                controller/'src/interface/constraint/SwingSafetyFilter.cpp',
                controller/'src/perceptive/synchronize/PlanarTerrainReceiver.cpp',
                workspace/'src/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp',
                workspace/'src/ocs2_ros2/submodules/plane_segmentation_ros2/convex_plane_decomposition/src/GridMapPreprocessing.cpp']
    manifest = {str(path.resolve().relative_to(workspace)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sources}
    manifest['trial.task.info'] = hashlib.sha256((config/'task.info').read_bytes()).hexdigest()
    binary = workspace/'install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so'
    if binary.is_file():
        manifest[str(binary.relative_to(workspace))] = hashlib.sha256(binary.read_bytes()).hexdigest()
    terrain_node = workspace/'install/convex_plane_decomposition_ros/lib/convex_plane_decomposition_ros/convex_plane_decomposition_ros_node'
    if terrain_node.is_file():
        manifest[str(terrain_node.relative_to(workspace))] = hashlib.sha256(terrain_node.read_bytes()).hexdigest()
    world = 'stair10cm'
    world_file = workspace/'src/quadruped_ros2_control/libraries/gz_quadruped_playground/worlds/stair10cm.sdf'
    if args.obstacle_height is not None or args.wrench_force:
        if args.obstacle_height is not None and not 0.0 < args.obstacle_height <= .3:
            p.error('--obstacle-height must be in (0, .3] metres')
        tree = ET.parse(world_file)
        obstacle = tree.getroot().find(".//model[@name='thin_obstacle']")
        if args.obstacle_height is not None:
            obstacle.find('pose').text = f'-1.2 0 {args.obstacle_height/2} 0 0 0'
            for size in obstacle.findall('.//geometry/box/size'):
                size.text = f'0.03 1.0 {args.obstacle_height}'
        if args.wrench_force:
            if not 0 < args.wrench_force <= 10 or not .005 <= args.wrench_duration <= .2:
                p.error('Force must be in (0,10] N and duration in [0.005,0.2] s')
            # Explicit plugins preserve the default server systems in this copy.
            world_element = tree.getroot().find('world')
            for filename, name in [('physics','Physics'), ('user-commands','UserCommands'),
                                   ('scene-broadcaster','SceneBroadcaster'), ('apply-link-wrench','ApplyLinkWrench')]:
                ET.SubElement(world_element, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        destination = args.output.resolve().with_suffix('.world.sdf')
        tree.write(destination, encoding='unicode')
        world = str(destination.with_suffix(''))
        world_file = destination
    obstacle = ET.parse(world_file).getroot().find(".//model[@name='thin_obstacle']")
    manifest['trial.obstacle'] = {
        'pose': [float(v) for v in obstacle.find('pose').text.split()],
        'collision_box_size': [float(v) for v in obstacle.find('.//collision/geometry/box/size').text.split()],
    }
    manifest['trial.wrench'] = {'force_z_N': -args.wrench_force, 'requested_duration_s': args.wrench_duration, 'link': 'robot::FL_knee'}
    manifest['trial.world.sdf'] = hashlib.sha256(world_file.read_bytes()).hexdigest()
    manifest['trial.spawn'] = {'x': args.x, 'y': args.y, 'yaw': args.yaw}
    args.output.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    logpath = args.output.with_suffix('.launch.log')
    with logpath.open('w') as log:
        launch = subprocess.Popen(['ros2','launch','ocs2_quadruped_controller','gazebo.launch.py',
                                   f'world:={world}', f'x:={args.x}', f'y:={args.y}', f'yaw:={args.yaw}',
                                   'gui:=false','rviz:=false'],
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+60
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError('Gazebo exited; see '+str(logpath))
                if re.search(r'Configured and activated .*ocs2_quadruped_controller',
                             logpath.read_text(errors='replace')):
                    break
                time.sleep(.2)
            else:
                raise RuntimeError('Controller activation timeout; see '+str(logpath))
            resource_limited = False
            probe = subprocess.Popen([sys.executable, str(Path(__file__).with_name('scenario_probe.py')),
                                    '--scenario', args.scenario, '--output', str(args.output),
                                    '--speed', str(args.speed), '--sim-seconds', str(args.sim_seconds),
                                    '--record-gz-poses', '--wrench-force', str(args.wrench_force),
                                    '--wrench-duration', str(args.wrench_duration)], env=env)
            while probe.poll() is None and launch.poll() is None:
                if available_memory_bytes() < memory_reserve:
                    resource_limited = True
                    print('Stopping trial: available memory fell below 1 GiB', flush=True)
                    break
                time.sleep(.2)
            if probe.poll() is None:
                probe.send_signal(signal.SIGINT)
                try:
                    probe.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    probe.kill()
                    probe.wait()
            outcome = 1 if resource_limited else probe.returncode

            trial_log = logpath.read_text(errors='replace')
            matches = re.findall(r'Swing replan .*? stats=(\{[^\n]*\})', trial_log)
            if args.output.exists():
                result = json.loads(args.output.read_text())
                if resource_limited:
                    result['result'] = 'resource_limit'
                result['replanning'] = json.loads(matches[-1]) if matches else None
                records = []
                for line in trial_log.splitlines():
                    match = re.search(r'Swing replan leg=(\d+) activation=(\S+)(?: remaining=(\S+) position_delta=(\S+))? outcome=(\d+) reason=(.*?) replan_ms=(\S+) stats=', line)
                    if match:
                        leg, ts, remaining, delta, code, reason, elapsed = match.groups()
                        records.append(dict(leg=int(leg), activation=float(ts), outcome=int(code),
                                            reason=reason, compute_ms=float(elapsed),
                                            remaining=float(remaining) if remaining else None,
                                            position_delta=float(delta) if delta else None))
                applies = []
                for match in re.finditer(
                        r'Swing replan applied leg=(\d+) activation=(\S+) now=(\S+) remaining=(\S+) p_error=(\S+) v_error=(\S+)',
                        trial_log):
                    leg, activation, now, remaining, p_error, v_error = match.groups()
                    applies.append(dict(leg=int(leg), activation=float(activation), applied=float(now),
                                        remaining=float(remaining), position_error=float(p_error),
                                        velocity_error=float(v_error)))
                args.output.with_suffix('.replans.json').write_text(
                    json.dumps({'decisions': records, 'applied': applies}, indent=2)+'\n')
                activation_checks = []
                for match in re.finditer(
                        r'Swing activation accepted=(\d+) leg=(\d+) activation=(\S+) now=(\S+) delay=(\S+) p_error=(\S+) v_error=(\S+) reason=(\S+)', trial_log):
                    accepted, leg, activation, now, delay, position, velocity, reason = match.groups()
                    activation_checks.append(dict(leg_pass=bool(int(accepted)), leg=int(leg), activation=float(activation),
                        actual_time=float(now), delay_s=float(delay), position_error_m=float(position),
                        velocity_error_mps=float(velocity), reason=reason))
                args.output.with_suffix('.activation.json').write_text(json.dumps(activation_checks, indent=2)+'\n')
                result['activation_leg_checks'] = len(activation_checks)
                result['activation_leg_rejections'] = sum(not row['leg_pass'] for row in activation_checks)
                result['replan_logged_decisions'] = len(records)
                result['replan_applied'] = len(applies)
                result['obstacle_height'] = args.obstacle_height
                result['safety_filter'] = args.safety_filter
                result['online_replan'] = args.online_replan
                result['safety_contact_force_deviation_override_N'] = args.safety_contact_force_deviation
                args.output.write_text(json.dumps(result, indent=2)+'\n')
            return outcome
        finally:
            stop_trial(launch, env['GZ_PARTITION'])


if __name__ == '__main__':
    raise SystemExit(main())
