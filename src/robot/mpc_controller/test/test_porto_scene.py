#!/usr/bin/env python3
"""Offline checks for static-map classification and conservative resampling."""
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np
from geometry_msgs.msg import Transform

spec = importlib.util.spec_from_file_location('porto_scene', Path(__file__).parents[1] / 'tools/porto_scene.py')
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


class PortoScene(unittest.TestCase):
    def test_configured_target_speed_and_limit(self):
        config = json.loads((Path(__file__).parents[1] / 'config/autodrive.json').read_text())
        self.assertEqual(scene.target_speed(config), config['target_speed_mps'])
        cap = config['speed_cap']
        for speed in (0., .6*cap, cap):
            self.assertEqual(scene.target_speed(dict(config, target_speed_mps=speed)), speed)
        for speed in (-.1, cap+.1, float('nan'), float('inf'), True, None):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                scene.target_speed(dict(config, target_speed_mps=speed))
        with self.assertRaises(ValueError):
            scene.target_speed(dict(config, target_speed_mps=0., speed_cap=0.))

    def test_known_map_and_rear_blind_region(self):
        root = Path(__file__).parents[2]
        cells, loops = scene.load_map(root / 'particle_filter/maps/porto.pgm')
        self.assertEqual(set(np.unique(cells)), {-1, 0, 100})
        self.assertEqual([len(loop) for loop in loops], [304, 188])
        prior = scene.prior_for_grid(cells, 40, 40, .025, (.3, 2.5, 0))
        self.assertIn(0, prior)
        self.assertIn(100, prior)
        live = np.full(prior.shape, -1, np.int8)
        merged = scene.merge_prior(live, prior)
        np.testing.assert_array_equal(merged, prior)
        live[:] = 0
        merged = scene.merge_prior(live, prior)
        self.assertTrue(np.all(merged[prior == 100] == 100))
        live[:] = 100
        self.assertTrue(np.all(scene.merge_prior(live, prior) == 100))

    def test_rotated_cell_supercover_includes_thin_wall_and_unknown(self):
        cells = np.zeros((4, 4), np.int8)
        cells[1, 1] = 100
        origin = (*scene.MAP_ORIGIN, np.pi/4)
        result = scene.prior_for_grid(cells, 8, 8, .025, origin)
        self.assertIn(100, result)
        self.assertIn(-1, result)
        with self.assertRaises(ValueError):
            scene.prior_for_grid(cells, 8, 8, .1, origin)
        # Every sampled touching source cell contributes, not only the output center.
        for y in range(8):
            for x in range(8):
                for dy in np.linspace(0, .025, 5):
                    for dx in np.linspace(0, .025, 5):
                        a, b = x*.025+dx, y*.025+dy
                        ix = int(np.floor((a-b)/np.sqrt(2)/.05))
                        iy = int(np.floor((a+b)/np.sqrt(2)/.05))
                        if 0 <= ix < 4 and 0 <= iy < 4 and cells[iy, ix] == 100:
                            self.assertEqual(result[y, x], 100)

    def test_tilt_limit(self):
        t = Transform()
        t.rotation.x, t.rotation.w = np.sin(.0314/2), np.cos(.0314/2)
        scene.planar_transform(t)
        t.rotation.x, t.rotation.w = np.sin(.06/2), np.cos(.06/2)
        with self.assertRaises(ValueError):
            scene.planar_transform(t)
        t.rotation.w = float('nan')
        with self.assertRaises(ValueError):
            scene.planar_transform(t)

    def test_centered_guide_is_finite_and_inside_measured_boundaries(self):
        root = Path(__file__).parents[2]
        cells, loops = scene.load_map(root / 'particle_filter/maps/porto.pgm')
        original = np.loadtxt(root / 'global_planner/assets/my_map_clean.csv', delimiter=',')
        guide = scene.centered_reference(cells, original)
        self.assertEqual(guide.shape, (147, 4))
        self.assertTrue(np.isfinite(guide).all())
        self.assertTrue(np.all(guide[:, 2:] > 0))
        for point in guide[:, :2]:
            inside = [scene.cv2.pointPolygonTest(loop.astype(np.float32), tuple(point), False) > 0
                      for loop in loops]
            self.assertNotEqual(*inside)


if __name__ == '__main__':
    unittest.main()
