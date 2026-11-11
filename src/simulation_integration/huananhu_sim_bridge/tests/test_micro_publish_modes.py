"""Regression tests for the 20261031-round micro publish parameters.

Counterexample-driven: each test also asserts what MUST NOT happen (fake
offsets, composite leakage into wedge40, wedge leakage into composite).
"""
import importlib.util
import math
import types
import sys
import unittest
from pathlib import Path

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

_source = Path(__file__).resolve().parents[1] / 'scripts/sim_lidar_to_livox.py'
sys.path.insert(0, str(_source.parent))
_spec = importlib.util.spec_from_file_location('sim_lidar_to_livox_under_test', _source)
module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(module)


def make_node(batch_count, publish_mode, messages):
    return types.SimpleNamespace(
        self_filter=None, mode='micro_scan', batch_count=batch_count,
        batch_period_ns=25_000_000, pending=[], batch_index=0,
        first_header=None, last_ns=None, scan_lines=16,
        v_min=-.610865, v_max=.610865,
        micro_publish_mode=publish_mode,
        pub=types.SimpleNamespace(publish=messages.append))


def bind(a):
    a.reset_scan = types.MethodType(module.LidarToLivox.reset_scan, a)
    a.process_cloud = types.MethodType(module.LidarToLivox.process_cloud, a)
    return a


def sample(a, ns, radius):
    h = Header(frame_id='navigation_laser')
    h.stamp.sec = ns // 1_000_000_000
    h.stamp.nanosec = ns % 1_000_000_000
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(['x', 'y', 'z', 'intensity'])]
    points = [(radius*math.cos(ang), radius*math.sin(ang), 0., 1.)
              for ang in [-2.4, -.8, .8, 2.4]]
    msg = point_cloud2.create_cloud(h, fields, points)
    module.LidarToLivox.on_cloud(a, msg)


class Wedge40Tests(unittest.TestCase):
    def test_each_acquisition_published_at_own_stamp_with_zero_offsets(self):
        msgs = []
        a = bind(make_node(4, 'wedge40', msgs))
        for i in range(4):
            sample(a, 1_000_000_000 + i*25_000_000, i + 1.)
        self.assertEqual(len(msgs), 4)
        stamps = [m.header.stamp.sec*10**9 + m.header.stamp.nanosec for m in msgs]
        self.assertEqual(stamps, [1_000_000_000 + i*25_000_000 for i in range(4)])
        for m in msgs:
            for p in m.points:
                self.assertEqual(p.offset_time, 0)

    def test_wedge40_must_not_emit_composite_offsets_or_delay(self):
        msgs = []
        a = bind(make_node(4, 'wedge40', msgs))
        sample(a, 2_000_000_000, 1.)
        self.assertEqual(len(msgs), 1)
        self.assertEqual(a.pending, [])
        self.assertTrue(all(p.offset_time == 0 for p in msgs[0].points))


class TwoBatchCompositeTests(unittest.TestCase):
    def test_two_batches_publish_one_composite_with_true_offsets(self):
        msgs = []
        a = bind(make_node(2, 'composite', msgs))
        for i in range(2):
            sample(a, 3_000_000_000 + i*25_000_000, i + 1.)
        self.assertEqual(len(msgs), 1)
        self.assertEqual(sorted({p.offset_time for p in msgs[0].points}),
                         [0, 25_000_000])
        self.assertEqual(msgs[0].point_num, len(msgs[0].points))

    def test_out_of_order_stamp_discards_partial_two_batch_scan(self):
        msgs = []
        a = bind(make_node(2, 'composite', msgs))
        sample(a, 4_000_000_000, 1.)
        sample(a, 4_000_000_000 - 50_000_000, 1.)
        sample(a, 4_000_000_000, 1.)
        sample(a, 4_000_000_000 + 25_000_000, 1.)
        self.assertEqual(len(msgs), 1)
        self.assertEqual(sorted({p.offset_time for p in msgs[0].points}),
                         [0, 25_000_000])


if __name__ == '__main__':
    unittest.main()
