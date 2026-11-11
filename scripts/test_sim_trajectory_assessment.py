#!/usr/bin/env python3
"""Independent numerical checks; no ROS, build or simulator required."""
import math
import unittest
import numpy as np
from sim_trajectory_evaluate import (chassis_to_base, compose, evaluate, interpolate_pose,
                                      inverse, rotate, slerp, yaw)


def pose(x=0., y=0., z=.1, yaw_angle=0.):
    return {'position':[x,y,z], 'orientation':[0.,0.,math.sin(yaw_angle/2),math.cos(yaw_angle/2)]}


class AssessmentTests(unittest.TestCase):
    def test_interpolation_uses_measurement_time(self):
        rows = [{'stamp_ns':1_000_000_000,'pose':pose()},
                {'stamp_ns':3_000_000_000,'pose':pose(2.,yaw_angle=math.pi/2)}]
        p = interpolate_pose(rows, [r['stamp_ns'] for r in rows], 2_000_000_000, 3_000_000_000)
        np.testing.assert_allclose(p['position'],[1.,0.,.1])
        self.assertAlmostEqual(yaw(p['orientation']),math.pi/4)
        with self.assertRaisesRegex(ValueError,'truth_gap_exceeded'):
            interpolate_pose(rows,[r['stamp_ns'] for r in rows],2_000_000_000,100)
        with self.assertRaisesRegex(ValueError,'truth_not_bracketed'):
            interpolate_pose(rows,[r['stamp_ns'] for r in rows],500_000_000,3_000_000_000)

    def test_rotated_chassis_lever_arm(self):
        # 90 degree pitch rotates downward chassis offset onto negative world X.
        p = {'position':[1.,2.,3.], 'orientation':[0.,math.sqrt(.5),0.,math.sqrt(.5)]}
        np.testing.assert_allclose(chassis_to_base(p)['position'],[.9,2.,3.],atol=1e-12)

    def test_antipodal_quaternions_and_yaw_wrap(self):
        q = pose(yaw_angle=math.radians(179))['orientation']
        np.testing.assert_allclose(slerp(q,-np.array(q),.5),q,atol=1e-12)
        q2 = pose(yaw_angle=math.radians(-179))['orientation']
        self.assertAlmostEqual(abs(yaw(slerp(q,q2,.5))),math.pi)

    def events(self, mode, moving=True, gauge=None, drift=0.):
        events = [{'kind':'metadata','mode':mode}]
        for i in range(5):
            stamp = (i+1)*1_000_000_000
            gt = pose(i*.25 if moving else 0.,yaw_angle=.1*i if moving else 0.)
            base = chassis_to_base(gt)
            estimate = compose(inverse(gauge),base) if gauge else base
            estimate['position'][0] += i*drift
            events.append({'kind':'truth','stamp_ns':stamp,'parent_frame':'world',
                           'child_frame':'huananhu/chassis','pose':gt})
            events.append({'kind':'estimate','stamp_ns':stamp,'parent_frame':'camera_init',
                           'child_frame':'base_footprint','pose':estimate,'tf_status':'OK','pose_map':base})
        return events

    def test_prior_origin_and_perfect_motion(self):
        out = evaluate(self.events('prior'))
        self.assertEqual(out['status'],'SCORED')
        self.assertEqual(out['coverage'],1.)
        self.assertAlmostEqual(out['metrics']['position_rmse_m'],0.)
        self.assertAlmostEqual(out['metrics']['truth_extent_m'],1.)

    def test_mapping_gauge_is_fixed_and_does_not_hide_drift(self):
        gauge = {'position':[5.,-2.,1.], 'orientation':[0.,math.sin(.2),0.,math.cos(.2)]}
        out = evaluate(self.events('mapping',gauge=gauge))
        self.assertEqual(out['status'],'SCORED')
        self.assertLess(out['metrics']['position_rmse_m'],1e-12)
        self.assertFalse(out['alignment']['per_frame_alignment'])
        drift = evaluate(self.events('mapping',gauge=gauge,drift=.05))
        self.assertGreater(drift['metrics']['position_rmse_m'],.10)
        self.assertAlmostEqual(drift['metrics']['final_position_error_m'],.20)

    def test_saved_local_uses_stamped_map_pose_then_fixed_gauge(self):
        gauge = pose(5.,-3.,1.,.7)
        events = self.events('saved-local',gauge=gauge)
        for row in events:
            if row['kind']=='estimate':
                row['pose_map']=row['pose']
                row['pose']=pose(99.,99.,99.)  # Raw new camera_init is a different frame.
        out=evaluate(events)
        self.assertEqual(out['status'],'SCORED')
        self.assertLess(out['metrics']['position_rmse_m'],1e-12)
        for row in events:
            if row['kind']=='estimate':row['tf_status']='UNAVAILABLE_AT_MEASUREMENT_STAMP'
        self.assertEqual(evaluate(events)['status'],'FAIL')

    def test_missing_tf_and_truth_do_not_become_zero_error(self):
        events = self.events('prior')
        for row in events:
            if row['kind']=='estimate':row['tf_status']='UNAVAILABLE_AT_MEASUREMENT_STAMP'
        out = evaluate(events)
        self.assertEqual(out['status'],'FAIL'); self.assertIsNone(out['metrics'])
        out = evaluate([r for r in events if r['kind']!='truth'])
        self.assertEqual(out['status'],'FAIL'); self.assertIsNone(out['metrics'])

    def test_static_data_and_contact_fail(self):
        out = evaluate(self.events('prior',moving=False))
        self.assertEqual(out['status'],'FAIL')
        self.assertEqual(out['motion_status'],'INSUFFICIENT_MOTION')
        events=self.events('prior')+[{'kind':'contact','contacts':2}]
        self.assertIn('body_contact_detected',evaluate(events)['reasons'])

    def test_rollback_is_not_hidden_by_sorting(self):
        events = self.events('prior')
        events += [{'kind':'truth','stamp_ns':1,'parent_frame':'world',
                    'child_frame':'huananhu/chassis','pose':pose()}]
        self.assertIn('clock_rollback_in_odometry',evaluate(events)['reasons'])


if __name__=='__main__':
    unittest.main()
