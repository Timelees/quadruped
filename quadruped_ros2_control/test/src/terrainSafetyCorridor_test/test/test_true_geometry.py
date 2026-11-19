import csv
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_true_geometry import LINKS, rotation, signed_clearance, true_feet


class TrueGeometryTests(unittest.TestCase):
    def test_model_relative_link_pose_is_composed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'poses.csv'
            with path.open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['sim_time', 'name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                writer.writerow([1, 'robot', 2, 3, 1, 0, 0, math.sqrt(.5), math.sqrt(.5)])
                for link in LINKS:
                    writer.writerow([1, link, 1, 0, 0, 0, 0, 0, 1])
            times, feet = true_feet(path)
            np.testing.assert_allclose(times, [1])
            np.testing.assert_allclose(feet[0], np.tile([2, 4, 1], (4, 1)), atol=1e-12)

    def test_rotated_box_signed_sphere_clearance(self):
        q = np.array([0, 0, math.sqrt(.5), math.sqrt(.5)])
        boxes = [(np.array([0, 0, 1]), rotation(q), np.array([1, .5, .5]))]
        values = signed_clearance(np.array([[0, 1.1, 1], [0, .9, 1]]), boxes)
        np.testing.assert_allclose(values, [.081, -.119], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
