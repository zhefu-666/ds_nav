import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial import ConvexHull,cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from cad_self_filter import CadSelfFilter,MeasuredJointHistory,closest_triangle_points

class SelfFilterTest(unittest.TestCase):
    def test_surface_only_near_obstacle_and_unknown_interior(self):
        f=object.__new__(CadSelfFilter);f.tolerance=.001;f.sensor=np.eye(4)
        vertices=np.array([[x,y,z] for x in (-.1,.1) for y in (-.1,.1) for z in (-.1,.1)])
        f.hulls={'yaw1':(ConvexHull(vertices).equations,vertices.min(0),vertices.max(0))}
        f.poses=lambda q:{'yaw1':np.eye(4)}
        hits,_=f.classify(np.array([[.1,0,0],[.12,0,0],[0,0,0],[.3,0,0]]),{})
        self.assertEqual(hits.tolist(),[True,False,False,False])

    def test_exact_triangle_distance_and_visual_filter(self):
        triangle=np.array([[[0.,0.,0.],[.1,0.,0.],[0.,.1,0.]]])
        points=np.array([[.02,.02,0.],[.02,.02,.02],[.1,.1,0.],[-.02,0.,0.]])
        actual=closest_triangle_points(np.repeat(triangle,4,axis=0),points)
        np.testing.assert_allclose(actual,[0.,.02,np.sqrt(.005),.02],atol=1e-12)
        f=object.__new__(CadSelfFilter);f.tolerance=.001;f.geometry='visual_surface';f.sensor=np.eye(4)
        f.visuals={'yaw1':(triangle,cKDTree(triangle.mean(1)),triangle.min((0,1)),triangle.max((0,1)))}
        self.install_visual_index(f,'yaw1',triangle)
        f.poses=lambda q:{'yaw1':np.eye(4)}
        hits,_=f.classify(points,{})
        self.assertEqual(hits.tolist(),[True,False,False,False])

    @staticmethod
    def install_visual_index(f, link, triangles):
        f.visual_bounds={link:(triangles.min(1),triangles.max(1))}
        radius=np.linalg.norm(triangles-triangles.mean(1)[:,None,:],axis=2).max()
        f.visual_trees={link:[(float(radius),np.arange(len(triangles)),cKDTree(triangles.mean(1)))]}

    def test_long_face_fallback_keeps_nearby_obstacle(self):
        f=self.cached_fixture()
        long_face=np.array([[[0.,0.,0.],[100.,0.,0.],[0.,100.,0.]]])
        small=np.array([[[.1,.1,.01],[.11,.1,.01],[.1,.11,.01]]])
        triangles=np.concatenate([long_face,np.repeat(small,128,axis=0)])
        f.visuals={'object':(triangles,cKDTree(triangles.mean(1)),triangles.min((0,1)),triangles.max((0,1)))}
        self.install_visual_index(f,'object',triangles)
        points=np.array([[.001,.001,0.],[.001,.001,.005]])
        self.assertNotIn(0,np.asarray(f.visuals['object'][1].query(points[0],k=128)[1]))
        for ray_ids in [None,[7,8],[7,8]]:
            hits,_=f.classify(points,{},ray_ids=ray_ids)
            self.assertEqual(hits.tolist(),[True,False])

    def cached_fixture(self):
        f=object.__new__(CadSelfFilter);f.tolerance=.001;f.geometry='visual_surface';f.sensor=np.eye(4)
        triangles=np.array([[[0.,0.,0.],[.1,0.,0.],[0.,.1,0.]]])
        f.visuals={'object':(triangles,cKDTree(triangles.mean(1)),triangles.min((0,1)),triangles.max((0,1)))}
        self.install_visual_index(f,'object',triangles)
        def poses(q):
            moved=np.eye(4);moved[2,3]=q.get('slide',0.)
            return {'yaw1':np.eye(4),'object':moved}
        f.poses=poses
        return f

    def test_same_ray_obstacle_does_not_reuse_hit(self):
        f=self.cached_fixture()
        self.assertTrue(f.classify(np.array([[.02,.02,0.]]),{},ray_ids=[123])[0][0])
        self.assertFalse(f.classify(np.array([[.02,.02,.02]]),{},ray_ids=[123])[0][0])
        # Cache hit must also fail for a near obstacle inside the link AABB.
        self.assertFalse(f.classify(np.array([[.08,.08,0.]]),{},ray_ids=[123])[0][0])

    def test_current_joint_pose_revalidates_triangle(self):
        f=self.cached_fixture();xyz=np.array([[.02,.02,0.]])
        self.assertTrue(f.classify(xyz,{'slide':0.},ray_ids=[7])[0][0])
        self.assertFalse(f.classify(xyz,{'slide':.03},ray_ids=[7])[0][0])
        self.assertTrue(f.classify(np.array([[.02,.02,.03]]),{'slide':.03},ray_ids=[7])[0][0])

    def test_cached_vs_uncached_and_slot_validation(self):
        f=self.cached_fixture()
        xyz=np.array([[.02,.02,0.],[.08,.08,0.],[.01,.01,.02]])
        baseline=f.classify(xyz,{})[0]
        for _ in range(3):
            np.testing.assert_array_equal(f.classify(xyz,{},ray_ids=[1,20,500])[0],baseline)
        with self.assertRaises(ValueError):f.classify(xyz,{},ray_ids=[1])

    def test_every_measured_joint_required(self):
        h=MeasuredJointHistory({'yaw1_joint','yaw2_joint','pitch1_drive_joint'})
        with self.assertRaises(ValueError):h.append(1,['yaw1_joint'],[0])
        names=['yaw1_joint','yaw2_joint','pitch1_drive_joint']
        h.append(1,names,[0,.2,.4]);h.append(1.05,names,[.1,.4,.8])
        q=h.at(1.025)
        self.assertAlmostEqual(q['yaw2_joint'],.3)
        self.assertAlmostEqual(q['pitch1_drive_joint'],.6)
        with self.assertRaises(ValueError):h.at(.9)
        h.append(1.3,names,[0,0,0])
        with self.assertRaises(ValueError):h.at(1.2)

if __name__=='__main__':unittest.main()
