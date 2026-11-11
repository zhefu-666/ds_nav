#!/usr/bin/env python3
"""Synthetic non-truth runtime checks with the actual CAD mount values."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sensor_state_math import JointHistory, mount_state, recover, add, cross, rotate, inv, mul, axis_q


class RecoveryTest(unittest.TestCase):
    def close(self, actual, expected):
        for a,b in zip(actual,expected):
            self.assertAlmostEqual(a,b,places=10)

    def synthesize(self, angle, rate, v=(0,0,0), w=(0,0,0)):
        state = mount_state(angle, rate)
        r,q,jw,jv = state
        pwb = (1.2,-2.3,.08)
        qwb = axis_q((0,0,1), .73)
        pwi = add(pwb, rotate(qwb,r))
        qwi = mul(qwb,q)
        vi = rotate(inv(q),add(add(v,cross(w,r)),jv))
        wi = rotate(inv(q),add(w,jw))
        result = recover(pwi,qwi,vi,wi,state)
        for actual,expected in zip(result,(pwb,qwb,v,w)):
            self.close(actual,expected)

    def test_yaw1_only_base_stationary(self):
        for a in (-2.,0.,1.3):
            self.synthesize(a,.8)

    def test_translation_rotation_lever(self):
        self.synthesize(.6,-.4,(.7,-.25,.02),(.03,.02,.8))

    def test_zero_mount_up(self):
        self.close(mount_state(0,0)[1],(0,0,0,1))
        self.close(mount_state(0,0)[0],(.0557597190051302,-.163594690492484,.198643863403111))

    def test_yaw2_has_no_input(self):
        # Sensor FK depends only on yaw1: no yaw2/pitch/body truth is consumed.
        import inspect
        self.assertEqual(list(inspect.signature(mount_state).parameters),['angle','rate'])
        self.synthesize(.3,.2)

    def test_interpolation_and_failure(self):
        h = JointHistory()
        h.append(1.,.2); h.append(1.05,.25)
        self.close(h.at(1.025),(.225,1.))
        for t in (.9,1.06):
            with self.assertRaises(ValueError): h.at(t)
        with self.assertRaises(ValueError): h.append(1.04,.3)
        h.append(1.3,.3)
        with self.assertRaises(ValueError): h.at(1.2)

if __name__ == '__main__': unittest.main()
