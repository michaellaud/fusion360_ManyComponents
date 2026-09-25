import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'TK_MeshBenchmark'))
import tk_core as tk  # noqa: E402


class MatrixTests(unittest.TestCase):
    def test_inverse(self):
        m = tk.from_trs([1, 2, 3], [0, 0, math.sin(0.3), math.cos(0.3)])
        self.assertTrue(tk.mat_close(tk.mat_mul(m, tk.mat_inverse(m)), tk.IDENTITY))

    def test_quat_z90(self):
        s = math.sqrt(0.5)
        m = tk.from_trs([0, 0, 0], [0, 0, s, s])
        self.assertTrue(tk.mat_close(tk.transform_points(m, [1, 0, 0]), [0, 1, 0]))

    def test_euler_matches_quat(self):
        s = math.sqrt(0.5)
        self.assertTrue(tk.mat_close(tk.from_euler_deg([0, 0, 0], [0, 0, 90]),
                                     tk.from_trs([0, 0, 0], [0, 0, s, s])))

    def test_yup_to_zup(self):
        self.assertTrue(tk.mat_close(tk.transform_points(tk.YUP_TO_ZUP, [0, 1, 0]), [0, 0, 1]))
        self.assertTrue(tk.mat_close(tk.mat_mul(tk.YUP_TO_ZUP, tk.ZUP_TO_YUP), tk.IDENTITY))


class ParseTests(unittest.TestCase):
    def test_frames_dict_column_major_mm(self):
        col = tk.transpose(tk.from_trs([10, 0, 0]))
        data = {'fps': 10, 'frames': [{'time': 0, 'transforms': {'A:1': col}},
                                      {'time': 0.1, 'transforms': {'A:1': col}}]}
        cfg = {'matrix_order': 'column', 'unit_scale_to_cm': 0.1}
        m = tk.convert_units_and_axes(tk.parse_motion(data, cfg), cfg)
        self.assertEqual(m.meta['detected_format'], 'frames')
        self.assertAlmostEqual(m.tracks['A:1']['mats'][1][3], 1.0)
        self.assertTrue(m.shared)

    def test_frames_list_objects_pos_quat(self):
        data = {'frames': [{'t': i * 0.5, 'objects': [{'id': 'p', 'position': [i, 0, 0],
                                                        'quaternion': [0, 0, 0, 1]}]} for i in range(3)]}
        m = tk.parse_motion(data)
        self.assertEqual(m.tracks['p']['mats'][2][3], 2.0)
        self.assertEqual(m.indices_at(0.7)['p'], 1)

    def test_tracks_trs(self):
        data = {'tracks': {'x': {'times': [0, 1], 'positions': [[0, 0, 0], [1, 1, 1]],
                                 'rotations': [[0, 0, 0, 1], [0, 0, 0, 1]]},
                           'y': {'times': [0, 2], 'matrices': [tk.IDENTITY, tk.IDENTITY]}}}
        m = tk.parse_motion(data)
        self.assertFalse(m.shared)
        self.assertEqual(m.duration, 2)
        self.assertEqual(m.indices_at(1.5), {'x': 1, 'y': 0})

    def test_unknown_raises(self):
        with self.assertRaises(ValueError):
            tk.parse_motion({'foo': 1})

    def test_relative_and_incremental(self):
        d = tk.from_trs([1, 0, 0])
        w0 = tk.from_trs([0, 5, 0])
        m = tk.Motion({'a': {'times': [0, 1], 'mats': [d, d]}})
        tk.finalize_tracks(m, {'a': w0}, {'transform_mode': 'incremental'})
        self.assertEqual(m.tracks['a']['mats'][1][3], 2.0)
        self.assertEqual(m.tracks['a']['mats'][1][7], 5.0)


class MatchTests(unittest.TestCase):
    def test_match(self):
        occs = [{'fullPathName': 'Base:1+Garra:1', 'name': 'Garra:1', 'componentName': 'Garra'},
                {'fullPathName': 'Base:1+Garra:2', 'name': 'Garra:2', 'componentName': 'Garra'},
                {'fullPathName': 'Mesa:1', 'name': 'Mesa:1', 'componentName': 'Mesa'}]
        ok, miss, amb = tk.match_ids(['Garra:1', 'garra_2', 'Mesa', 'Garra', 'X', 'custom'],
                                     occs, {'custom': 'Mesa:1'})
        self.assertEqual(ok['Garra:1'], 'Base:1+Garra:1')
        self.assertEqual(ok['garra_2'], 'Base:1+Garra:2')
        self.assertEqual(ok['Mesa'], 'Mesa:1')
        self.assertEqual(ok['custom'], 'Mesa:1')
        self.assertIn('Garra', amb)
        self.assertEqual(miss, ['X'])


class StatsTests(unittest.TestCase):
    def test_summary(self):
        s = tk.summarize(list(range(1, 101)))
        self.assertAlmostEqual(s['mean'], 50.5)
        self.assertAlmostEqual(s['p95'], 95.05)

    def test_drops(self):
        dc = tk.DropCounter(10)
        for t in (0.0, 0.1, 0.4, 0.5):
            dc.presented(t)
        self.assertEqual(dc.dropped, 2)

    def test_sim_time_loop(self):
        self.assertAlmostEqual(tk.sim_time(2.5, 0, 2), 0.5)
        self.assertAlmostEqual(tk.sim_time(2.5, 0, 2, loop=False), 2.0)


if __name__ == '__main__':
    unittest.main()
