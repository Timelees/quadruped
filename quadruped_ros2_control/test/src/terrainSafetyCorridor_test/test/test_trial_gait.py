import os
from pathlib import Path
import re
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_trial import override_task


class TrialGaitTest(unittest.TestCase):
    def test_duration_override_is_isolated_and_keeps_other_gaits(self):
        workspace = next(path for path in Path(__file__).resolve().parents
                         if (path/'src/quadruped_ros2_control/descriptions').is_dir())
        description=workspace/'src/quadruped_ros2_control/descriptions/cyberdog_description'
        original=(description/'config/ocs2/gait.info').read_bytes()
        with tempfile.TemporaryDirectory() as folder:
            config=override_task(dict(os.environ),description,Path(folder)/'trial.json',{},.6)
            copied=(config/'gait.info').read_text()
            block=re.search(r'(?ms)^trot\s*\{.*?switchingTimes\s*\{([^}]*)',copied)[1]
            times=[float(value) for value in re.findall(r'\[\d+\]\s+(\S+)',block)]
            self.assertEqual(times,[0.,.6,1.2])
            self.assertEqual(copied.split('\nstanding_trot\n',1)[1],original.decode().split('\nstanding_trot\n',1)[1])
        self.assertEqual((description/'config/ocs2/gait.info').read_bytes(),original)


if __name__=='__main__':
    unittest.main()
