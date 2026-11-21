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


def override_task(env, source, output, settings, swing_duration=None):
    """Expose an isolated robot description via the ament index."""
    if not settings and swing_duration is None:
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
    if swing_duration is not None:
        gait = config/'gait.info'
        contents, count = re.subn(r'(?ms)(^trot\s*\{.*?switchingTimes\s*\{)[^}]*(\})',
            lambda match: match[1]+f'\n     [0] 0.0\n     [1] {swing_duration:g}\n     [2] {2*swing_duration:g}\n   '+match[2],
            gait.read_text())
        if count != 1:
            raise ValueError(f'Expected one trot switchingTimes block, found {count}')
        gait.write_text(contents)
    env['AMENT_PREFIX_PATH'] = str(overlay)+os.pathsep+env.get('AMENT_PREFIX_PATH', '')
    return config

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', required=True, choices=['trot', 'stairs', 'platform', 'obstacle', 'trench', 'piles', 'course', 'flat', 'lateral'])
    p.add_argument('--lateral-direction', type=int, choices=[-1, 1], default=1)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seed', type=int, help='Gazebo random seed, paired across modes')
    p.add_argument('--domain', type=int, default=82)
    p.add_argument('--x', type=float, default=None)
    p.add_argument('--y', type=float, default=None)
    p.add_argument('--yaw', type=float, default=0.0, help='Spawn heading in radians')
    p.add_argument('--speed', type=float, default=.5)
    p.add_argument('--swing-duration', type=float, help='Isolated trot half-period [s]; keeps each trial contact schedule fixed')
    p.add_argument('--sim-seconds', type=float, default=100.)
    p.add_argument('--target', type=float, help='Override scenario destination for a local obstacle experiment')
    p.add_argument('--swing-update-method', choices=['frozen','direct','admitted'], help='Controlled causal ablation; direct bypasses state admission')
    p.add_argument('--controlled-map-source', choices=['perception','world-truth'], default='perception', help='World-truth is an explicitly labelled mechanism experiment, not perception validation')
    p.add_argument('--controlled-reveal', action='store_true', help='Mask the second bar in controller maps until an LF swing trigger')
    p.add_argument('--reveal-at-x', type=float, default=1.4)
    p.add_argument('--reveal-phase', type=float, default=.25)
    p.add_argument('--reveal-unknown', action='store_true', help='Unknown certificate before reveal; default is a deliberately wrong floor map')
    p.add_argument('--reveal-require-crossing', action='store_true', help='Release only when the current LF swing forecast spans the revealed bar')
    p.add_argument('--hold-seconds', type=float, default=3., help='Continuous destination trot hold')
    p.add_argument('--world', type=Path, help='Static SDF world; an exact copy is saved per trial')
    p.add_argument('--wrench-at-x', type=float, help='Trigger during LF swing after reaching this world X coordinate')
    p.add_argument('--obstacle-height', type=float, help='Obstacle top height in metres; writes an isolated world copy')
    p.add_argument('--safety-filter', choices=['off', 'hocbf', 'full'])
    p.add_argument('--online-replan', choices=['on', 'off'])
    p.add_argument('--mrt-delay-start', type=float, default=12., help='Begin delay injection after trot establishment [controller s]')
    p.add_argument('--mrt-transport-delay', type=float, help='Actual MRT in-flight package hold in simulation seconds')
    p.add_argument('--corridor-height-error', type=float)
    p.add_argument('--corridor-map-margin', type=float)
    p.add_argument('--corridor-vertical-extent', type=float,help='Isolated corridor search extent; preserves tracking and collision margins')
    p.add_argument('--replan-prediction-delay', type=float,
                   help='Planner prediction horizon before candidate activation [s]; not MRT transport delay')
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
    if args.output.exists():
        p.error('Output already exists; use a new trial name to preserve previous data')
    if args.wrench_at_x is not None and not math.isfinite(args.wrench_at_x):
        p.error('--wrench-at-x must be finite')
    if not 0 <= args.domain <= 232:
        p.error('--domain must be in [0,232] for the default Fast DDS port mapping')
    if args.seed is not None and not 0 <= args.seed <= 4294967295:
        p.error('--seed must fit an unsigned 32-bit integer')
    # Keep headroom for the desktop; do not wait until swap is exhausted.
    memory_reserve = 1024 * 1024 * 1024
    if available_memory_bytes() < memory_reserve:
        p.error('Less than 1 GiB memory available; Gazebo was not started')
    if args.controlled_reveal and args.scenario not in ('course','obstacle'): p.error('Controlled reveal requires course or obstacle')
    if not 0 <= args.reveal_phase <= .75: p.error('--reveal-phase must be in [0,.75]')
    if args.swing_duration is not None and (not math.isfinite(args.swing_duration) or not .2 <= args.swing_duration <= .8):
        p.error('--swing-duration must be in [.2,.8] s')
    if args.target is not None and not math.isfinite(args.target): p.error('--target must be finite')
    if args.corridor_vertical_extent is not None and not .021<args.corridor_vertical_extent<=.3:
        p.error('--corridor-vertical-extent must be in (.021,.3] metres')
    if args.y is None:
        args.y = -3.0 if args.scenario == 'course' else (-8.0 if args.scenario == 'flat' else 0.0)
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
    if args.corridor_vertical_extent is not None:
        settings['corridorVerticalExtent']=str(args.corridor_vertical_extent)
    if args.replan_prediction_delay is not None:
        if not math.isfinite(args.replan_prediction_delay) or not 0 <= args.replan_prediction_delay <= .1:
            p.error('--replan-prediction-delay must be in [0,.1] s')
        settings['swingReplanActivationDelay'] = str(args.replan_prediction_delay)
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
    for value, key, limit in [(args.mrt_transport_delay, 'mrtTransportDelay', .25),
            (args.corridor_height_error, 'corridorHeightError', .15), (args.corridor_map_margin, 'corridorMapMargin', .10)]:
        if value is not None:
            if not math.isfinite(value) or not 0 <= value <= limit: p.error(f'{key} must be in [0,{limit}]')
            settings[key] = str(value)
    if not math.isfinite(args.hold_seconds) or args.hold_seconds < 3: p.error('--hold-seconds must be >= 3')
    if args.mrt_transport_delay is not None:
        if not math.isfinite(args.mrt_delay_start) or args.mrt_delay_start < 0: p.error('--mrt-delay-start must be finite and nonnegative')
        settings['mrtTransportStartTime'] = str(args.mrt_delay_start)
    if args.swing_update_method is not None:
        if not args.controlled_reveal or args.safety_filter != 'off':
            p.error('--swing-update-method requires --controlled-reveal and --safety-filter off')
        settings['swingOnlineReplanEnabled'] = 'false' if args.swing_update_method == 'frozen' else 'true'
        settings['swingReplanDirectTestOnly'] = 'true' if args.swing_update_method == 'direct' else 'false'
    config = override_task(env, description, args.output, settings, args.swing_duration)
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
                controller/'launch/gazebo.launch.py', controller/'src/FSM/StateOCS2.cpp', controller/'src/interface/constraint/SwingTrajectoryPlanner.cpp',
                controller/'src/interface/constraint/SwingOnlineReplanner.cpp',
                controller/'src/interface/constraint/SwingSafetyFilter.cpp',
                controller/'src/perceptive/synchronize/PlanarTerrainReceiver.cpp',
                workspace/'src/ocs2_ros2/mpc/ocs2_mpc/src/MRT_BASE.cpp',
                workspace/'src/quadruped_ros2_control/hardwares/gz_quadruped_hardware/src/gz_system.cpp',
                workspace/'src/quadruped_ros2_control/hardwares/gz_quadruped_hardware/include/gz_quadruped_hardware/actuator.hpp',
                controller/'src/control/TargetManager.cpp', controller/'src/control/CtrlComponent.cpp',
                controller/'launch/elevation_mapping.launch.py',
                controller/'include/ocs2_quadruped_controller/interface/constraint/SwingOnlineReplanner.h',
                controller/'include/ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h',
                Path(__file__).with_name('controlled_terrain.py'),
                workspace/'src/ocs2_ros2/submodules/plane_segmentation_ros2/convex_plane_decomposition/src/GridMapPreprocessing.cpp']
    manifest = {str(path.resolve().relative_to(workspace)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sources}
    manifest['trial.task.info'] = hashlib.sha256((config/'task.info').read_bytes()).hexdigest()
    manifest['trial.gait.info'] = hashlib.sha256((config/'gait.info').read_bytes()).hexdigest()
    binary = workspace/'install/ocs2_quadruped_controller/lib/ocs2_quadruped_controller/libocs2_quadruped_controller.so'
    if binary.is_file():
        manifest[str(binary.relative_to(workspace))] = hashlib.sha256(binary.read_bytes()).hexdigest()
    motor_binary = workspace/'install/gz_quadruped_hardware/lib/libgz_quadruped_plugins.so'
    if motor_binary.is_file():
        manifest[str(motor_binary.relative_to(workspace))] = hashlib.sha256(motor_binary.read_bytes()).hexdigest()
    terrain_node = workspace/'install/convex_plane_decomposition_ros/lib/convex_plane_decomposition_ros/convex_plane_decomposition_ros_node'
    if terrain_node.is_file():
        manifest[str(terrain_node.relative_to(workspace))] = hashlib.sha256(terrain_node.read_bytes()).hexdigest()
    world = 'stair10cm'
    world_file = workspace/'src/quadruped_ros2_control/libraries/gz_quadruped_playground/worlds/stair10cm.sdf'
    if args.world is not None:
        world_file = args.world.resolve(strict=True)
    if args.obstacle_height is not None and not 0.0 < args.obstacle_height <= .3:
        p.error('--obstacle-height must be in (0, .3] metres')
    tree = ET.parse(world_file)
    controlled_bars=[]
    if args.controlled_reveal and args.scenario == 'course':
        for index in range(4):
            bar=tree.getroot().find(f".//model[@name='safety_lane_bar_{index}']")
            if bar is None: p.error('World-truth intervention requires the validated four-bar lane')
            pose=list(map(float,bar.find('pose').text.split()))
            size=list(map(float,bar.find('.//collision/geometry/box/size').text.split()))
            if len(pose)!=6 or len(size)!=3 or not all(math.isfinite(v) for v in pose+size) or min(size)<=0 or \
                    abs(pose[2]-size[2]/2)>1e-8 or any(abs(v)>1e-8 for v in pose[3:]) or not -3.7<=pose[1]<=-2.3:
                p.error('World-truth intervention requires axis-aligned bars resting on the ground in the test lane')
            controlled_bars.append([pose[0],size[0],size[2],pose[1],size[1]])
        manifest['trial.controlled_bars'] = controlled_bars
    obstacle = tree.getroot().find(".//model[@name='thin_obstacle']")
    if args.obstacle_height is not None:
        obstacle.find('pose').text = f'-1.2 0 {args.obstacle_height/2} 0 0 0'
        for size in obstacle.findall('.//geometry/box/size'):
            size.text = f'0.03 1.0 {args.obstacle_height}'
    if args.controlled_reveal and args.scenario == 'obstacle':
        obstacle = tree.getroot().find(".//model[@name='thin_obstacle']")
        pose=list(map(float,obstacle.find('pose').text.split()))
        size=list(map(float,obstacle.find('.//collision/geometry/box/size').text.split()))
        controlled_bars=[[pose[0],size[0],pose[2]+size[2]/2,pose[1],size[1]]]
        manifest['trial.controlled_bars']=controlled_bars
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
    manifest['trial.wrench'] = {'force_z_N': -args.wrench_force, 'requested_duration_s': args.wrench_duration,
                              'link': 'robot::FL_knee', 'trigger_x_m': args.wrench_at_x}
    manifest['trial.world.sdf'] = hashlib.sha256(world_file.read_bytes()).hexdigest()
    manifest['trial.map_sampling'] = dict(near_obstacles_m=.02, elsewhere_m=.10, minimum_time_interval_s=1.0)
    manifest['trial.seed'] = args.seed
    reveal_centre = controlled_bars[0 if args.scenario == 'obstacle' else 1][0] if controlled_bars else 1.8
    manifest['trial.reveal'] = dict(enabled=args.controlled_reveal, phase=args.reveal_phase,
        trigger_x=args.reveal_at_x, unknown=args.reveal_unknown,
        require_crossing=args.reveal_require_crossing,
        mask=[reveal_centre-.2,reveal_centre+.2,*([-.65,.65] if args.scenario == 'obstacle' else [-3.7,-2.3])], source=args.controlled_map_source,
        note='test-only certificate reveal; planar region topology is retained from perception')
    manifest['trial.spawn'] = {'x': args.x, 'y': args.y, 'yaw': args.yaw}
    manifest['trial.route'] = dict(scenario=args.scenario, target_override=args.target,
        normalized_speed=args.speed, lateral_direction=args.lateral_direction,
        hold_seconds=args.hold_seconds, maximum_sim_seconds=args.sim_seconds,
        swing_duration_s=args.swing_duration if args.swing_duration is not None else .4)
    args.output.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    logpath = args.output.with_suffix('.launch.log')
    with logpath.open('w') as log:
        launch = subprocess.Popen(['ros2','launch','ocs2_quadruped_controller','gazebo.launch.py',
                                   f'world:={world}', f'x:={args.x}', f'y:={args.y}', f'yaw:={args.yaw}',
                                   'gui:=false','rviz:=false', f'controlled_terrain:={str(args.controlled_reveal).lower()}'] + ([f'seed:={args.seed}'] if args.seed is not None else []),
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
                                    '--swing-duration', str(args.swing_duration if args.swing_duration is not None else .4),
                                    '--hold-seconds', str(args.hold_seconds), '--record-gz-poses', '--wrench-force', str(args.wrench_force),
                                    '--wrench-duration', str(args.wrench_duration), '--lane-y', str(args.y),
                                    '--lateral-direction', str(args.lateral_direction)] +
                                    (['--target',str(args.target)] if args.target is not None else []) +
                                    (['--wrench-at-x', str(args.wrench_at_x)] if args.wrench_at_x is not None else []) +
                                    (['--controlled-reveal','--controlled-map-source',args.controlled_map_source,'--reveal-at-x',str(args.reveal_at_x),'--reveal-phase',str(args.reveal_phase),
                                      '--known-course-bars',json.dumps(controlled_bars)] + (['--reveal-require-crossing'] if args.reveal_require_crossing else []) +
                                     (['--reveal-unknown'] if args.reveal_unknown else []) if args.controlled_reveal else []), env=env)
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
                    json.dumps({'decisions': records, 'applied': applies,
                        'applied_envelopes': [dict(leg=int(m[0]), activation=float(m[1]),
                            velocity_mps=float(m[2]), acceleration_mps2=float(m[3]), jerk_mps3=float(m[4]),
                            end_error_s=float(m[5]), touchdown_velocity_z_mps=float(m[6])) for m in re.findall(
                            r'Swing applied envelope leg=(\d+) activation=(\S+) v_max=(\S+) a_max=(\S+) j_max=(\S+) end_error=(\S+) touchdown_vz=(\S+)',
                            trial_log)]}, indent=2)+'\n')
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
                result['swing_update_method'] = args.swing_update_method
                result['controlled_reveal'] = args.controlled_reveal
                result['mrt_transport_start_s'] = args.mrt_delay_start if args.mrt_transport_delay is not None else None
                result['mrt_transport_delay_s'] = args.mrt_transport_delay
                result['hold_seconds'] = args.hold_seconds
                result['safety_contact_force_deviation_override_N'] = args.safety_contact_force_deviation
                args.output.write_text(json.dumps(result, indent=2)+'\n')
            return outcome
        finally:
            stop_trial(launch, env['GZ_PARTITION'])


if __name__ == '__main__':
    raise SystemExit(main())
