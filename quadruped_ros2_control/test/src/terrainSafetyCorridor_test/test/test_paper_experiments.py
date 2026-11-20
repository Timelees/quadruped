import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paper_experiments import contact_events, confirm_contact_geometry, scan_summary


class ContactTimingTest(unittest.TestCase):
    def fixture(self):
        rows=[]
        for i in range(121):
            t=i*.005
            mode=15 if t<.1 or t>=.5 else 7  # LF swings .1 to .5
            row={'sim_time':t,'mode':mode}
            for leg in ('LF','RF','LH','RH'):
                row['sensor_'+leg]=0 if leg=='LF' and .1<=t<.51 else 10
            rows.append(row)
        return rows

    def test_detects_late_physical_contact(self):
        event=contact_events(self.fixture())[0]
        self.assertAlmostEqual(event['timing_error_s'],.01)
        self.assertAlmostEqual(event['phase_error_swing_fraction'],.025)

    def test_single_force_spike_is_not_touchdown(self):
        rows=self.fixture()
        rows[70]['sensor_LF']=10
        self.assertAlmostEqual(contact_events(rows)[0]['contact_s'],.51)

    def test_dropout_does_not_invent_contact(self):
        rows=self.fixture()
        rows=[row for row in rows if not .46<row['sim_time']<.51]
        self.assertIsNone(contact_events(rows)[0]['contact_s'])

    def test_airborne_force_onset_is_not_confirmed_touchdown(self):
        rows=[{'sim_time':.51,'ros_time':1.,'LF_pd_x':0.,'LF_pd_y':0.,'LF_pd_z':.019}]
        event={'leg':'LF','contact_s':.51,'deadline_s':.5}
        positions=np.zeros((1,4,3));positions[:,:,2]=.10
        with patch('analyze_true_geometry.true_feet',return_value=(np.array([1.]),positions)), \
                patch('analyze_true_geometry.world_boxes',return_value=[]):
            confirm_contact_geometry(Path('/tmp/example.json'),rows,[event])
        self.assertFalse(event['target_contact_confirmed'])
        positions[:,:,2]=.019
        with patch('analyze_true_geometry.true_feet',return_value=(np.array([1.]),positions)), \
                patch('analyze_true_geometry.world_boxes',return_value=[]):
            confirm_contact_geometry(Path('/tmp/example.json'),rows,[event])
        self.assertTrue(event['target_contact_confirmed'])


class ScanSummaryTest(unittest.TestCase):
    def test_safe_retention_is_not_collision_repair(self):
        rows=[]
        for case, before, after in [('0',.01,.02),('1',-.01,.02),('2',-.01,float('nan'))]:
            for method, clearance in [('Frozen',before),('Admitted',after)]:
                rows.append(dict(case_id=case,method=method,clearance_m=str(clearance),admitted='1',outcome='1'))
        summary=scan_summary(rows)['Admitted']
        self.assertEqual(summary['safe_sampled'],2)
        self.assertEqual(summary['rescued_frozen_collisions'],1)
        self.assertEqual(summary['frozen_colliding_conditions'],2)
        self.assertEqual(summary['unobserved_clearance_accepted'],1)


if __name__=='__main__':
    unittest.main()
