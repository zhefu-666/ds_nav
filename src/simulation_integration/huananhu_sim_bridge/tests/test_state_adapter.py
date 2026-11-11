"""No simulator, executor, build or ROS graph required; run with Humble sourced."""
import copy
import math
import sys
import time
from pathlib import Path
import types
import unittest

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

# Execute the saved source without generating bytecode.
source = Path(__file__).resolve().parents[1] / 'scripts/sim_state_adapter.py'
sys.path.insert(0, str(source.parent))
module = types.ModuleType('adapter_under_test')
exec(compile(source.read_text(), str(source), 'exec'), module.__dict__)


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))

    sendTransform = publish


class Stamp:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return Stamp(self.nanoseconds - other.nanoseconds)


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.a = types.SimpleNamespace(
            last_clock_ns=None, epoch_reset=False, guard=None, last_clock_wall=time.monotonic(),
            previous_measurement=None, mode="prior", mount_link="chassis",
            lidar_pub=Sink(),
            latest=None, latest_stamp=None, parent='camera_init', child='base_footprint',
            offset=[0., 0., -0.1], stale_after=.5, gicp_state_stale_after=1.,
            gicp_state='localized', gicp_state_stamp=Stamp(self.now), require_map=False,
            get_clock=lambda: types.SimpleNamespace(now=lambda: Stamp(self.now)),
            tf_broadcaster=Sink(), base_pub=Sink(), valid_pub=Sink(), status_pub=Sink(),
        )
        self.a.publish_status = types.MethodType(module.SimStateAdapter.publish_status, self.a)
        self.msg = Odometry()
        self.msg.header.frame_id = 'camera_init'
        self.msg.child_frame_id = 'body'
        self.msg.header.stamp.sec = 10
        self.msg.pose.pose.orientation.w = 1.
        self.msg.twist.twist.angular.y = 2.
        module.SimStateAdapter.on_odom(self.a, self.msg)

    def tick(self):
        module.SimStateAdapter.on_timer(self.a)

    def test_fresh_received_measurement_precedes_timeout_check(self):
        self.msg.twist.twist.angular.y = 0.
        self.a.guard = module.TrackingGuard()
        self.tick()
        # Timer was delayed, but a continuous, fresh measurement already arrived.
        self.now += 550_000_000
        self.msg.header.stamp.nanosec = 450_000_000
        module.SimStateAdapter.on_odom(self.a, self.msg)
        self.tick()
        self.assertIsNone(self.a.guard.reason)
        self.assertTrue(self.a.valid_pub.messages[-1].data)

    def test_real_measurement_gap_remains_latched(self):
        self.msg.twist.twist.angular.y = 0.
        self.a.guard = module.TrackingGuard()
        self.tick()
        self.now += 650_000_000
        self.msg.header.stamp.nanosec = 600_000_000
        module.SimStateAdapter.on_odom(self.a, self.msg)
        self.tick()
        self.assertEqual(self.a.guard.reason, 'measurement_gap')
        self.assertFalse(self.a.valid_pub.messages[-1].data)

    def test_no_new_measurement_still_latches_stale(self):
        self.msg.twist.twist.angular.y = 0.
        self.a.guard = module.TrackingGuard()
        self.tick()
        self.now += 550_000_000
        self.tick()
        self.assertEqual(self.a.guard.reason, 'measurement_stale')
        self.assertFalse(self.a.valid_pub.messages[-1].data)

    def test_reference_point_and_require_map_false(self):
        self.tick()
        base = self.a.base_pub.messages[-1]
        self.assertAlmostEqual(base.pose.pose.position.z, -.1)
        self.assertAlmostEqual(base.twist.twist.linear.x, -.2)
        self.assertEqual(base.header.stamp, self.msg.header.stamp)
        self.assertEqual(base.child_frame_id, 'base_footprint')
        self.assertTrue(self.a.valid_pub.messages[-1].data)
        self.assertEqual(self.msg.pose.pose.position.z, 0.)

    def test_old_measurement_recent_arrival(self):
        self.a.latest.header.stamp.sec = 8
        self.tick()
        self.assertFalse(self.a.valid_pub.messages[-1].data)
        self.assertEqual(len(self.a.base_pub.messages), 0)

    def test_future_measurement(self):
        self.a.latest.header.stamp.sec = 11
        self.tick()
        self.assertFalse(self.a.valid_pub.messages[-1].data)

    def test_reset_latches_invalid_after_new_messages(self):
        self.tick()
        self.now = 1_000_000_000
        self.tick()
        self.msg.header.stamp.sec = 1
        module.SimStateAdapter.on_odom(self.a, self.msg)
        module.SimStateAdapter.on_gicp_state(self.a, String(data='localized'))
        self.tick()
        self.assertFalse(self.a.valid_pub.messages[-1].data)
        self.assertTrue(self.a.epoch_reset)

    def test_gicp_rejection_and_stale(self):
        for state, stamp in [('converging', self.now), ('localized', self.now-2_000_000_000)]:
            self.a.gicp_state, self.a.gicp_state_stamp = state, Stamp(stamp)
            self.tick()
            self.assertFalse(self.a.valid_pub.messages[-1].data)

    def test_bad_frame_and_nan(self):
        self.msg.header.frame_id = 'map'
        module.SimStateAdapter.on_odom(self.a, self.msg)
        self.assertIsNone(self.a.latest)

    def test_invalid_frame_then_fresh_frame_does_not_unlock(self):
        self.a.guard = module.TrackingGuard()
        self.a.guard.mark_valid()
        self.msg.header.frame_id = 'wrong_frame'
        module.SimStateAdapter.on_odom(self.a, self.msg)
        self.msg.header.frame_id = 'camera_init'
        self.msg.header.stamp.nanosec = 100_000_000
        self.now += 100_000_000
        module.SimStateAdapter.on_odom(self.a, self.msg)
        self.tick()
        self.assertEqual(self.a.guard.reason, 'unexpected_odometry_frame')
        self.assertFalse(self.a.valid_pub.messages[-1].data)
        self.msg.header.frame_id = 'camera_init'
        self.msg.pose.pose.position.x = math.nan
        module.SimStateAdapter.on_odom(self.a, self.msg)
        self.assertIsNone(self.a.latest)


if __name__ == '__main__':
    unittest.main()
