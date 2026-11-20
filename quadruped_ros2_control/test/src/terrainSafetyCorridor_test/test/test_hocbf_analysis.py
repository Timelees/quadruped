import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_hocbf_trials import terrain_height, wilson


class AnalysisChecks(unittest.TestCase):
    def test_wilson_includes_boundaries(self):
        self.assertAlmostEqual(wilson(0,5)[0],0.)
        self.assertAlmostEqual(wilson(5,5)[1],1.)
        self.assertGreater(wilson(5,5)[0],.5)
        self.assertIsNone(wilson(0,0))

    def test_vertical_ray_rotated_box_and_plane(self):
        angle=.2
        rotation=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        boxes=[(np.array([0.,0.,.2]),rotation,np.array([1.,1.,.1]))]
        heights=terrain_height(np.array([[0.,0.],[3.,0.]]),boxes)
        self.assertAlmostEqual(heights[0],.2+.1/np.cos(angle))
        self.assertEqual(heights[1],0.)


if __name__=='__main__':
    unittest.main()
