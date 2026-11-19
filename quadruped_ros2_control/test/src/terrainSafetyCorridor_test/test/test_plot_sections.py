"""Run with python3 -m unittest discover -s <package>/test; no ROS/build needed."""
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from plot_corridor_sections import section_polygon, plot_scene


class SectionTests(unittest.TestCase):
    def test_oblique_halfspace_section(self):
        # Exact XZ rectangle with a cut corner x+z<=1.5. The ay term is
        # nonzero, so omitting the fixed-y translation changes the polygon.
        dtype = [(name, float) for name in ('ax', 'ay', 'az', 'b_geom')]
        planes = np.array([(1, 0, 0, 1), (-1, 0, 0, 0), (0, 0, 1, 1),
                           (0, 0, -1, 0), (1, 2, 1, 2.5)], dtype=dtype)
        polygon = section_polygon(planes, 'b_geom', .5, (-2, 2, -2, 2))
        expected = np.array([(0, 0), (1, 0), (1, .5), (.5, 1), (0, 1)])
        self.assertEqual(len(polygon), len(expected))
        for v in expected:
            self.assertLess(np.linalg.norm(polygon-v, axis=1).min(), 1e-10)
        for plane in planes:
            self.assertTrue(np.all(plane['ax']*polygon[:, 0] + plane['ay']*.5 +
                                   plane['az']*polygon[:, 1] <= plane['b_geom'] + 1e-10))

    def test_contact_coordinate_translation_preserves_halfspaces(self):
        # Moving just the trajectory makes a misleading plot: normals with a
        # vertical component need the same b -> b-a_z*r translation.
        normal = np.array([.2, -.3, .9])
        centre = np.array([.25, .1, .18])
        radius, bound = .019, .3
        contact = centre - np.array([0, 0, radius])
        translated = bound - normal[2] * radius
        self.assertAlmostEqual(normal @ centre - bound, normal @ contact - translated)

    def test_exported_multiswing_supports_and_bar_height(self):
        data = Path(__file__).resolve().parents[1] / 'data/swing_sequences'
        if not (data / 'stairs_swings.csv').exists():
            self.skipTest('Run the production multi-swing exporter first')
        from plot_corridor_sections import load
        for scene, count in [('flat', 1), ('bar', 1), ('pillars', 4), ('stairs', 3)]:
            manifest = np.atleast_1d(np.genfromtxt(data / f'{scene}_swings.csv',
                delimiter=',', names=True, dtype=None, encoding='utf8'))
            self.assertEqual(len(manifest), count)
            previous = None
            for stem in manifest['stem']:
                curve = load(data / f'{stem}_trajectory.csv')
                metrics = load(data / f'{stem}_metrics.csv')
                terrain = load(data / f'{stem}_terrain.csv')
                radius = metrics['foot_radius'][0]
                self.assertEqual(metrics['output_certificate'][0], 0)
                self.assertGreaterEqual(metrics['optimized_min_clearance'][0], -1e-8)
                self.assertLessEqual(metrics['corridor_max_violation'][0], 1e-9)
                for index in (0, -1):
                    xy = np.array([curve['ref_x'][index], curve['ref_y'][index]])
                    distance = (terrain['x']-xy[0])**2 + (terrain['y']-xy[1])**2
                    support = terrain['z'][np.argmin(distance)]
                    self.assertAlmostEqual(curve['ref_z'][index]-radius, support, places=8)
                    self.assertAlmostEqual(curve['opt_z'][index]-radius, support, places=8)
                if previous is not None:
                    self.assertAlmostEqual(previous, curve['ref_x'][0])
                previous = curve['ref_x'][-1]
                if scene == 'bar':
                    self.assertAlmostEqual(terrain['z'].max(), .10, places=7)

    def test_legacy_data_cannot_be_presented_as_new_evidence(self):
        data = Path(__file__).resolve().parents[1] / 'data/final'
        if not (data / 'bar_trajectory.csv').exists() or (data / 'bar_metrics.csv').exists():
            self.skipTest('Historical export fixture is absent or was regenerated')
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, 'Legacy exports'):
                plot_scene(data, Path(directory), 'bar')
            self.assertEqual(list(Path(directory).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
