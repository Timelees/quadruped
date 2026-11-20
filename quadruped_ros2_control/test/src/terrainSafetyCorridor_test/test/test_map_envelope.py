import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_map_envelope import analyze


class EnvelopeTest(unittest.TestCase):
    def test_upper_patch_and_unknown_are_not_pointwise_promotions(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'sample.json'
            path.with_suffix('.manifest.json').write_text(json.dumps({'trial.map_sampling':{'near_obstacles_m':.02}}))
            path.with_suffix('.task.info').write_text('corridorHeightError 0.005\n')
            path.with_suffix('.world.sdf').write_text('''<sdf version="1.9"><world name="test"><model name="bar"><static>true</static>
              <pose>1.1 -3 0.02 0 0 0</pose><link name="box"><collision name="shape"><geometry><box><size>.06 1.4 .04</size></box></geometry></collision></link></model></world></sdf>''')
            with path.with_suffix('.certified_map.csv').open('w') as stream:
                writer=csv.writer(stream);writer.writerow(['ros_time','x','y','certified_height_m'])
                for time,high in [(1,.04),(2,.01),(3,float('nan'))]:
                    for dx in (0,1):
                        for dy in (0,1): writer.writerow([time,1.08+dx*.02,-3+dy*.02,high if dx==dy==1 else 0.])
            result=analyze(path)
            self.assertEqual(result['known_patches'],2)
            self.assertEqual(result['unknown_patches'],1)
            self.assertEqual(result['violating_patch_samples'],1)
            self.assertAlmostEqual(result['true_minus_prism_minus_budget_m']['maximum'],.025)


if __name__=='__main__': unittest.main()
