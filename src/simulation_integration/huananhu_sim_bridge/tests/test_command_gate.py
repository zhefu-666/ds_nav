import copy
from pathlib import Path
import types
import unittest
from unittest.mock import patch
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
m=types.ModuleType('gate');p=Path(__file__).resolve().parents[1]/'scripts/sim_command_gate.py';exec(compile(p.read_text(),str(p),'exec'),m.__dict__)
class GateTests(unittest.TestCase):
    def setUp(self):
        self.messages=[];self.a=types.SimpleNamespace(upright=True,healthy_at={'base':10.,'imu':10.,'joints':10.},mode='manual',command=[1.,0.,1.],command_at=10.,valid=True,valid_at=10.,contact_at=-1e9,posture_ready=None,posture_at=0.,output=Twist(),last=9.98,pub=types.SimpleNamespace(publish=lambda x:self.messages.append(copy.deepcopy(x))),status=types.SimpleNamespace(publish=lambda x:None))
    def tick(self,now=10.):
        with patch.object(m.time,'monotonic',return_value=now):m.CommandGate.tick(self.a)
    def test_timeout_clears_replay(self):
        self.tick(11.);self.assertIsNone(self.a.command);self.assertEqual(self.messages[-1].linear.x,0.)
    def test_invalid_clears_replay(self):
        self.a.valid=False;self.tick();self.assertIsNone(self.a.command)
    def test_mode_switch_clears_old_command(self):
        m.CommandGate.on_mode(self.a,String(data='auto'));self.assertIsNone(self.a.command);self.assertEqual(self.messages[-1].linear.x,0.)
    def test_limit_and_ramp(self):
        self.tick();self.assertAlmostEqual(self.messages[-1].linear.x,.006)
        self.assertAlmostEqual(self.messages[-1].angular.z,.016)
    def test_tilt_or_missing_sensor_stops(self):
        self.a.upright=False;self.tick();self.assertIsNone(self.a.command)
        self.a.upright=True;self.a.command=[.1,0.,0.];self.a.healthy_at.pop('imu');self.tick();self.assertIsNone(self.a.command)
    def test_wrong_source_and_nonfinite(self):
        self.a.command=None;m.CommandGate.on_cmd(self.a,Twist(),'auto');self.assertIsNone(self.a.command)
        t=Twist();t.linear.x=float('nan');m.CommandGate.on_cmd(self.a,t,'manual');self.assertIsNone(self.a.command)
if __name__=='__main__':unittest.main()
