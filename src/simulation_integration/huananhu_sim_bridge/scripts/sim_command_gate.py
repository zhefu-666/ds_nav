#!/usr/bin/env python3
"""Single simulation actuator owner. No truth subscriptions."""
import math
import time
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock,ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool,String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,JointState
from rclpy.qos import qos_profile_sensor_data


class CommandGate(Node):
    def __init__(self):
        super().__init__('sim_command_gate')
        self.mode='disabled';self.command=None;self.command_at=0.;self.valid=False;self.valid_at=0.
        self.output=Twist();self.last=time.monotonic()
        self.healthy_at={};self.upright=False
        self.contact_at=-1e9;self.posture_ready=None;self.posture_at=0.
        self.pub=self.create_publisher(Twist,'/sim/actuator_cmd_vel',10)
        self.status=self.create_publisher(String,'/sim/control_status',10)
        self.create_subscription(Odometry,'/sim/base_odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Imu,'/livox/imu',lambda m:self.on_sensor('imu'),qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',lambda m:self.on_sensor('joints'),qos_profile_sensor_data)
        self.create_subscription(Contacts,'/assessment/body_contacts',self.on_contact,qos_profile_sensor_data)
        self.create_subscription(Bool,'/sim/posture_ready',self.on_posture_ready,10)
        self.create_subscription(Bool,'/sim/localization_valid',self.on_valid,10)
        self.create_subscription(String,'/sim/control_mode',self.on_mode,10)
        self.create_subscription(Twist,'/sim/manual_cmd_vel',lambda m:self.on_cmd(m,'manual'),10)
        self.create_subscription(Twist,'/sim/auto_cmd_vel',lambda m:self.on_cmd(m,'auto'),10)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_sensor(self,name):
        self.healthy_at[name]=time.monotonic()

    def on_contact(self,msg):
        if not msg.contacts:return
        self.contact_at=time.monotonic()
        self.command=None

    def on_posture_ready(self,msg):
        self.posture_ready=msg.data;self.posture_at=time.monotonic()

    def on_odom(self,msg):
        q=msg.pose.pose.orientation
        norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        # z-axis projection rejects excessive roll/pitch in both modes.
        self.upright=(math.isfinite(norm) and abs(norm-1.)<.01 and
                      1.-2.*(q.x*q.x+q.y*q.y)>math.cos(.35))
        self.on_sensor('base')
        if not self.upright:self.command=None

    def on_valid(self,msg):
        self.valid=msg.data;self.valid_at=time.monotonic()
        if not self.valid:self.command=None

    def on_mode(self,msg):
        if msg.data not in ('disabled','manual','auto','estop'):return
        if msg.data!=self.mode:
            self.mode=msg.data;self.command=None;self.output=Twist()
            self.pub.publish(self.output)

    def on_cmd(self,msg,source):
        if source!=self.mode:return
        values=[msg.linear.x,msg.linear.y,msg.angular.z]
        if not all(math.isfinite(x) for x in values):self.command=None;return
        if not self.valid or time.monotonic()-self.valid_at>.5:
            self.command=None;return
        self.command=values;self.command_at=time.monotonic()

    def tick(self):
        now=time.monotonic();dt=min(now-self.last,.05);self.last=now
        reason='active'
        if self.mode not in ('manual','auto'):reason=self.mode
        elif not self.valid or now-self.valid_at>.5:reason='localization_invalid_or_stale'
        elif not self.upright or any(now-self.healthy_at.get(k,0.)>.5 for k in ('base','imu','joints')):reason='tilt_or_sensor_stale'
        # Posture interlock only applies while the posture controller is alive;
        # without it the articulated model is not running (e.g. low_profile_robot).
        elif self.posture_ready is not None and now-self.posture_at<=.5 and not self.posture_ready:reason='posture_not_ready'
        elif now-self.contact_at<.35:reason='body_contact_stop'
        elif self.command is None or now-self.command_at>.3:reason='command_timeout'
        if reason!='active':
            self.command=None;self.output=Twist()
        else:
            x,y,w=self.command;norm=math.hypot(x,y)
            if norm>.3:x*=.3/norm;y*=.3/norm
            w=max(-.5,min(.5,w))
            old=[self.output.linear.x,self.output.linear.y,self.output.angular.z]
            targets=[x,y,w];step=[.3*dt,.3*dt,.8*dt]
            new=[a+max(-s,min(s,b-a)) for a,b,s in zip(old,targets,step)]
            self.output.linear.x,self.output.linear.y,self.output.angular.z=new
        self.pub.publish(self.output);self.status.publish(String(data=reason))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=CommandGate()
    stopping = False

    def request_stop(_signum, _frame):
        # Keep signals out of pybind message conversion and preserve final zero.
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:pass
    finally:
        node.pub.publish(Twist());node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
