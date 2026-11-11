"""Rendered wheel surfaces must use measured joints; environment stays intact."""
import sys
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/simulation_integration/huananhu_sim_bridge/scripts'))
from cad_self_filter import CadSelfFilter


class WheelFilterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.filter = CadSelfFilter(ROOT/'src/simulation_integration/huananhu_sim_resources')

    def test_measured_wheel_surfaces_removed(self):
        f = self.filter
        for angle in (0., .7):
            q = dict.fromkeys(f.required, 0.)
            for wheel in range(1, 5):
                q[f'wheel_{wheel}_joint'] = angle
            poses = f.poses(q)
            sensor = poses['yaw1'] @ f.sensor
            for wheel in range(1, 5):
                name = f'wheel_{wheel}'
                point = f.visuals[name][0][0].mean(axis=0)
                transform = np.linalg.inv(sensor) @ poses[name]
                xyz = (transform[:3,:3] @ point + transform[:3,3])[None,:]
                hits, counts = f.classify(xyz, q)
                self.assertTrue(hits[0], (name, angle, counts))

    def test_long_face_not_lost_by_centroid_shortlist(self):
        from scipy.spatial import cKDTree
        f = CadSelfFilter.__new__(CadSelfFilter)
        small = np.array([[0.,0.,.01],[.03,0.,.01],[0.,.03,.01]])
        large = np.array([[0.,0.,0.],[2.,0.,0.],[0.,2.,0.]])
        triangles = np.concatenate([np.tile(small[None,:,:], (128,1,1)), large[None,:,:]])
        f.geometry, f.tolerance, f.sensor = 'visual_surface', .001, np.eye(4)
        f.poses = lambda _: {'yaw1':np.eye(4), 'part':np.eye(4)}
        f.visuals = {'part':(triangles,cKDTree(triangles.mean(1)),triangles.min((0,1)),triangles.max((0,1)))}
        f.visual_bounds = {'part':(triangles.min(1),triangles.max(1))}
        radius = float(np.linalg.norm(triangles-triangles.mean(1)[:,None,:],axis=2).max())
        f.visual_trees = {'part':[(radius,np.arange(len(triangles)),cKDTree(triangles.mean(1)))]}
        hits, _ = f.classify(np.array([[.001,.001,0.],[.001,.001,.004]]), {})
        self.assertEqual(hits.tolist(), [True, False])

    def test_missing_wheel_measurement_rejected(self):
        q = dict.fromkeys(self.filter.required, 0.)
        del q['wheel_1_joint']
        with self.assertRaisesRegex(ValueError, 'missing_measured_joints'):
            self.filter.classify(np.array([[3.,0.,0.]]), q)

    def test_environment_outside_robot_retained(self):
        q = dict.fromkeys(self.filter.required, 0.)
        hits, _ = self.filter.classify(np.array([[3.,0.,0.],[0.,0.,3.]]), q)
        self.assertFalse(hits.any())


if __name__ == '__main__':
    unittest.main()
