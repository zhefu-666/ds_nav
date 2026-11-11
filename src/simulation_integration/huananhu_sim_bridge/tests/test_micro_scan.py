"""Check real input timestamps, changing samples and dropped-batch reset."""
from pathlib import Path
import types
import sys
import unittest
import math
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

source = Path(__file__).resolve().parents[1] / 'scripts/sim_lidar_to_livox.py'
sys.path.insert(0, str(source.parent))
module = types.ModuleType('lidar_under_test')
exec(compile(source.read_text(), str(source), 'exec'), module.__dict__)


class MicroTests(unittest.TestCase):
    def setUp(self):
        self.messages = []
        self.a = types.SimpleNamespace(self_filter=None, mode='micro_scan', batch_count=4,
            batch_period_ns=25_000_000, pending=[], batch_index=0,
            first_header=None, last_ns=None, scan_lines=16,
            v_min=-.610865,v_max=.610865,
            micro_publish_mode='composite',
            pub=types.SimpleNamespace(publish=self.messages.append))

        self.a.reset_scan = types.MethodType(module.LidarToLivox.reset_scan, self.a)
        self.a.process_cloud = types.MethodType(module.LidarToLivox.process_cloud, self.a)

    def sample(self, ns, radius):
        h=Header(frame_id='navigation_laser');h.stamp.sec=ns//1_000_000_000;h.stamp.nanosec=ns%1_000_000_000
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(['x','y','z','intensity'])]
        points=[(radius*math.cos(angle),radius*math.sin(angle),0.,1.) for angle in [-2.4,-.8,.8,2.4]]
        msg=point_cloud2.create_cloud(h,fields,points)
        module.LidarToLivox.on_cloud(self.a,msg)

    def test_actual_batches_no_truth_compensation(self):
        for i in range(4):self.sample(1_000_000_000+i*25_000_000,i+1.)
        self.assertEqual(len(self.messages),1)
        msg=self.messages[0]
        self.assertEqual([p.offset_time for p in msg.points],[0,25_000_000,50_000_000,75_000_000])
        for i,p in enumerate(msg.points):self.assertAlmostEqual(math.hypot(p.x,p.y),i+1.,places=5)
        self.assertEqual(msg.timebase,1_000_000_000)

    def test_missed_rays_not_encoded_as_returns(self):
        self.sample(1_000_000_000,float('inf'))
        self.assertEqual(len(self.a.pending),0)

    def test_gap_and_clock_rollback_discard_partial_scan(self):
        self.sample(1_000_000_000,1.)
        self.sample(1_100_000_000,2.)
        self.assertEqual(self.a.batch_index,1)
        self.sample(500_000_000,3.)
        self.assertEqual(self.a.batch_index,1)
        self.assertEqual(len(self.messages),0)

if __name__=='__main__':unittest.main()
