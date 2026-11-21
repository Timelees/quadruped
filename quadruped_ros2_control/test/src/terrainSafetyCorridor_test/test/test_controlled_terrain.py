import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controlled_terrain import mask_strip, known_course_layer, swing_spans_bar, set_update_release


class ControlledTerrainTest(unittest.TestCase):
    def test_release_does_not_mutate_elevation_or_duplicate_layer(self):
        m=S(layers=['elevation'],data=[S(data=[.1,.2])])
        set_update_release(m,False)
        self.assertEqual(m.data[0].data,[.1,.2])
        self.assertEqual(m.data[1].data,[0.,0.])
        set_update_release(m,True)
        self.assertEqual(len(m.layers),2)
        self.assertEqual(m.data[1].data,[1.,1.])

    def test_reveal_requires_free_footholds_on_both_sides(self):
        self.assertFalse(swing_spans_bar(.59,[],.62,.01))
        self.assertFalse(swing_spans_bar(.59,[.63],.62,.01))
        self.assertFalse(swing_spans_bar(.63,[.67],.62,.01))
        self.assertTrue(swing_spans_bar(.59,[.65,.66],.62,.01))
        self.assertTrue(swing_spans_bar(-1.10,[-1.24,-1.32],-1.20,.03,.019))
        self.assertFalse(swing_spans_bar(-1.10,[-1.21],-1.20,.03,.019))

    def test_rolling_mask_preserves_outside_and_unknown_evidence(self):
        m = S(info=S(resolution=.1, length_x=.4, length_y=.4,
                     pose=S(position=S(x=0., y=0.))), outer_start_index=2, inner_start_index=1,
              layers=['elevation', 'elevation_certified', 'smooth_planar', 'unrelated'],
              data=[S(layout=S(dim=[S(label='column_index')]), data=[.2]*16) for _ in range(4)])
        changed = mask_strip(m, (.09,.16,.09,.16), unknown=True)
        # World (+.15,+.15) is logical (0,0), buffer (2,1), index 6.
        self.assertEqual(changed, 3)
        self.assertTrue(math.isnan(m.data[1].data[6]))
        self.assertEqual(m.data[0].data[6], 0.)
        self.assertTrue(all(v == .2 for i,v in enumerate(m.data[1].data) if i != 6))
        self.assertTrue(all(v == .2 for v in m.data[3].data))

    def test_known_course_intervention_only_changes_certificate(self):
        m = S(info=S(resolution=.02, length_x=.2, length_y=.2,
                     pose=S(position=S(x=1.8, y=-3.))), outer_start_index=3, inner_start_index=2,
              layers=['elevation', 'elevation_certified'],
              data=[S(layout=S(dim=[S(label='column_index')]), data=[.001]*100) for _ in range(2)])
        known_course_layer(m)
        self.assertTrue(all(h==.001 for h in m.data[0].data))
        self.assertEqual(max(m.data[1].data),.07)
        self.assertEqual(min(m.data[1].data),0.)
        known_course_layer(m, [(1.8,.01,.06)])
        self.assertEqual(max(m.data[1].data),.06)
        self.assertTrue(all(h==.001 for h in m.data[0].data))
        known_course_layer(m,[(1.8,.01,.06,-2.76,.16)])
        self.assertEqual(max(m.data[1].data),0.)


if __name__ == '__main__':
    unittest.main()
