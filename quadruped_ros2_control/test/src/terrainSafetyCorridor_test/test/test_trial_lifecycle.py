"""Run with python3 -m unittest discover -s <package>/test -p test_trial_lifecycle.py."""
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run_trial
from run_trial import stop_trial, trial_processes


class TrialLifecycleTests(unittest.TestCase):
    def test_invalid_domain_prevents_launch(self):
        with patch.object(sys, 'argv', ['run_trial.py', '--scenario', 'trot', '--domain', '233',
                                       '--output', '/tmp/not_started.json']), \
                patch.object(run_trial.subprocess, 'Popen') as spawn, patch('sys.stderr'):
            with self.assertRaises(SystemExit) as error:
                run_trial.main()
            self.assertEqual(error.exception.code, 2)
            spawn.assert_not_called()

    def test_low_memory_prevents_launch(self):
        with patch.object(sys, 'argv', ['run_trial.py', '--scenario', 'trot', '--output', '/tmp/not_started.json']), \
                patch.object(run_trial, 'available_memory_bytes', return_value=0), \
                patch.object(run_trial.subprocess, 'Popen') as spawn, \
                patch('sys.stderr'):
            with self.assertRaises(SystemExit) as error:
                run_trial.main()
            self.assertEqual(error.exception.code, 2)
            spawn.assert_not_called()

    def test_reaps_separate_group_after_launch_exit_and_preserves_other_trial(self):
        partition = 'terrain_cleanup_test_' + uuid.uuid4().hex
        env = dict(os.environ, GZ_PARTITION=partition)
        other = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                                 env=dict(env, GZ_PARTITION=partition + '_other'))
        launch = subprocess.Popen(
            [sys.executable, '-c',
             'import subprocess,sys; '
             'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"],'
             'start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); '
             'print(p.pid,flush=True)'], env=env, stdout=subprocess.PIPE, text=True)
        try:
            child = int(launch.stdout.readline())
            launch.wait(timeout=5)
            self.assertIn(child, trial_processes(partition))
            stop_trial(launch, partition)
            self.assertEqual(trial_processes(partition), [])
            self.assertIsNone(other.poll())
        finally:
            stop_trial(launch, partition)
            launch.stdout.close()
            other.terminate()
            other.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
