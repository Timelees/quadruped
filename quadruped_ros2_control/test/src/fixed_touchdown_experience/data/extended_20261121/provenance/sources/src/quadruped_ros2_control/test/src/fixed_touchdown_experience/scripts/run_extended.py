#!/usr/bin/env python3
"""Run the five-figure protocol after building and sourcing the ROS2 workspace.
A fallen Gazebo pilot is retained; it never becomes a physical success rate.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--figure',type=Path,required=True)
    p.add_argument('--domain',type=int,default=179);p.add_argument('--seed',type=int,default=20261121)
    a=p.parse_args();ws=next(v for v in Path(__file__).resolve().parents if (v/'src/quadruped_ros2_control/controllers').is_dir())
    scripts=ws/'src/quadruped_ros2_control/test/src/fixed_touchdown_experience/scripts';trial=scripts.parents[1]/'terrainSafetyCorridor_test/run_trial.py'
    data=a.data.resolve();figure=a.figure.resolve()
    env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',MPLCONFIGDIR='/tmp/fixed_touchdown_mpl')
    def run(command,environment=env,allow_failure=False):
        result=subprocess.run(list(map(str,command)),env=environment,cwd=ws)
        if result.returncode and not allow_failure:raise subprocess.CalledProcessError(result.returncode,command)
        return result.returncode
    run([sys.executable,scripts/'prepare_sweep.py','--world',ws/'src/quadruped_ros2_control/libraries/gz_quadruped_playground/worlds/stair10cm.sdf','--output',data])
    pilot_env=dict(env,FIXED_TOUCHDOWN_EXPORT=str(data/'wbc.jsonl'))
    code=run([sys.executable,trial,'--scenario','obstacle','--world',data/'pilot.sdf','--output',data/'gazebo_pilot',
        '--domain',a.domain,'--seed',a.seed,'--speed','.4','--sim-seconds','100','--target','-4.35',
        '--safety-filter','full','--online-replan','on'],pilot_env,True)
    (data/'pilot_exit.json').write_text(json.dumps(dict(exit_code=code))+'\n')
    if not (data/'wbc.jsonl.swings.jsonl').exists():raise RuntimeError('No actual swings recorded; stopping, no invented fallback data')
    analysis_env=dict(env,PYTHONNOUSERSITE='1')
    run(['/usr/bin/python3',scripts/'analyze_extended.py',data],analysis_env)
    duration=json.loads((data/'runtime_schedule.json').read_text())['median_s']
    # Snap the verified decimal gait time (the production corridor requires exact endpoints).
    duration=round(duration,9)
    assert abs(duration-.4)<1e-8,'This frozen figure protocol expects a verified 0.4 s swing; update the bin labels for another gait.'
    run(['ros2','run','fixed_touchdown_experience','feasibility_sweep',data/'gazebo_pilot.task.info',data/'wbc.jsonl.swings.jsonl',data/'sweep',duration])
    run(['ros2','run','fixed_touchdown_experience','hull_export',data/'sweep/accepted.jsonl',data/'sweep/hulls_instantiated.jsonl'])
    run([sys.executable,scripts/'verify_extended.py',data]);run([sys.executable,scripts/'plot_extended.py',data,'--output',figure])
    run([sys.executable,scripts/'archive_extended.py','--data',data,'--figure',figure])
    print('Raw data:',data,'Figures:',figure)

if __name__=='__main__':main()
