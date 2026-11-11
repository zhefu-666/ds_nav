import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from sensor_state_math import TrackingGuard

class TrackingGuardTest(unittest.TestCase):
    def ready(self):
        g=TrackingGuard()
        self.assertIsNone(g.observe(1.,(0,0,0),(0,0,0,1),(0,0,0),(0,0,0)))
        g.mark_valid()
        return g

    def test_normal(self):
        g=self.ready()
        self.assertIsNone(g.observe(1.1,(.03,0,0),(0,0,0,1),(.3,0,0),(0,0,.2)))
        self.assertIsNone(g.check_time(1.2))

    def test_lost_return_never_unlocks(self):
        g=self.ready()
        self.assertEqual(g.check_time(1.6),'measurement_stale')
        self.assertEqual(g.observe(1.61,(0,0,0),(0,0,0,1),(0,0,0),(0,0,0)),'measurement_stale')
        g.mark_valid()
        self.assertIsNotNone(g.reason)

    def test_gap_on_new_measurement(self):
        g=self.ready()
        self.assertEqual(g.observe(1.6,(0,0,0),(0,0,0,1),(0,0,0),(0,0,0)),'measurement_gap')

    def test_pose_jump_even_zero_reported_speed(self):
        g=self.ready()
        self.assertEqual(g.observe(1.1,(275,195,0),(0,0,0,1),(0,0,0),(0,0,0)),'base_pose_jump')
        self.assertEqual(g.observe(1.2,(0,0,0),(0,0,0,1),(0,0,0),(0,0,0)),'base_pose_jump')

    def test_speed_and_yaw_limits(self):
        g=self.ready()
        self.assertEqual(g.observe(1.1,(0,0,0),(0,0,0,1),(15,0,0),(0,0,0)),'base_speed_limit')
        g=self.ready()
        self.assertEqual(g.observe(1.1,(0,0,0),(0,0,0,1),(0,0,0),(0,0,2)),'base_yaw_rate_limit')

    def test_pause_uses_simulation_time(self):
        g=self.ready()
        # Arbitrarily long wall pause leaves simulation stamp unchanged.
        for _ in range(100):self.assertIsNone(g.check_time(1.))
        self.assertIsNone(g.observe(1.01,(0,0,0),(0,0,0,1),(0,0,0),(0,0,0)))

if __name__=='__main__':unittest.main()
