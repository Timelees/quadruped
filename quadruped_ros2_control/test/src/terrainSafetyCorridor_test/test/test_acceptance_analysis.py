import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_acceptance import certificate_durations, reveal_window, masked_unknown_samples


class CertificateDurationTest(unittest.TestCase):
    def test_unknown_mask_window_excludes_delivery_transient_and_released_map(self):
        events = [dict(event='mask_applied',ros_time=1.),
                  dict(event='map_released',received_ros_time=2.)]
        rows = []
        for time,state in ((1.1,1),(1.5,2),(1.6,0),(2.1,1)):
            row = dict(ros_time=time,mode=7,LF_p_x=.5,LF_p_y=.5,
                       LF_certificate_state=state,LF_safety_status=0)
            rows.append(row)
        result = masked_unknown_samples(rows, events, (0.,1.,0.,1.))
        self.assertEqual(result['observed_swing_leg_samples'],2)
        self.assertEqual(result['certified_samples'],0)
        self.assertEqual(result['certificate_states'],{'2':1,'0':1})

    def test_reveal_does_not_borrow_other_leg_or_later_swing(self):
        trigger = dict(controller_time=1.1, liftoff=1.)
        rows = [dict(sim_time=t, LF_p_x=x) for t,x in ((1.,.5),(1.2,.6),(1.4,.8))]
        applied = [dict(leg=leg, applied=t) for leg,t in ((0,1.05),(0,1.2),(1,1.3),(0,1.5))]
        events = [dict(leg=0,time=1.25),dict(leg=0,time=1.6)]
        log = 'Swing replan leg=0 activation=1.3 remaining=.1 outcome=4 reason=QP infeasible replan_ms=1\n'
        log += 'Swing replan leg=0 activation=1.5 remaining=.1 outcome=1 reason=ok replan_ms=1\n'
        result = reveal_window(trigger, rows, applied, events, [], log)
        self.assertEqual(result['observed_LF_x_range_m'],[.5,.6])
        self.assertEqual(result['applied'],[applied[1]])
        self.assertEqual(result['direct'],[events[0]])
        self.assertEqual(len(result['planner_attempts']),1)
        self.assertEqual(result['planner_attempts'][0]['reason'],'QP infeasible')

    def test_interleaved_ros_log_preserves_the_target_swing_rejection(self):
        log = '[ruby sim-4] Swing replan leg=[INFO] [123.4] [controller]: transport release\n'
        log += '[ruby sim-4] 0 activation=1.25 remaining=.15 outcome=3 reason=state outside correction gate replan_ms=0.200458 stats={}\n'
        result = reveal_window(dict(controller_time=1.1,liftoff=1.), [], [], [], [], log)
        self.assertEqual(result['planner_attempts'], [dict(time_s=1.25,outcome=3,
            reason='state outside correction gate',computation_ms=.200458)])
        self.assertEqual(result['unparsed_planner_records_total'],0)

    def test_recorded_gait_duration_defines_the_target_swing_window(self):
        events = [dict(leg=0, applied=time) for time in (1.45,1.65)]
        result = reveal_window(dict(controller_time=1.1,liftoff=1.,swing_duration_s=.6),
            [], events, [], [], '')
        self.assertAlmostEqual(result['end_s'],1.6)
        self.assertEqual(result['applied'],events[:1])

    def test_diagnostic_gap_is_not_certified_or_filled(self):
        rows = [dict(sim_time=time, **{leg+'_certificate_state': state
                for leg in ('LF','RF','LH','RH')})
                for time,state in ((1.,2),(1.005,2),(1.010,1),(1.110,2),(1.115,0))]
        result = certificate_durations(rows)
        self.assertAlmostEqual(result['observed_leg_seconds']['2'],.060)
        self.assertEqual(result['omitted_gaps'],1)
        self.assertAlmostEqual(result['omitted_gap_seconds'],.1)
        lf = [r for r in result['uncertified_intervals'] if r['leg']=='LF']
        self.assertEqual(len(lf),2)
        self.assertAlmostEqual(lf[0]['end_s'],1.010)


if __name__ == '__main__':
    unittest.main()
