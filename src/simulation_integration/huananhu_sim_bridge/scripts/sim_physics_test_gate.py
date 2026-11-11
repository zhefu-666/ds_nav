#!/usr/bin/env python3
"""DIAGNOSTIC physics-test actuator gate. Not a localization implementation.

This gate exists only so the drawing-dimension tunnel fixtures can be exercised
without the formal Point-LIO/GICP gate. It uses Gazebo ground truth for bounded
physical feedback (tilt and test status), never publishes /sim/localization_valid
or any map TF, and is mutually exclusive with sim_command_gate.py in launch.
The single ROS -> Gazebo actuator channel stays /sim/actuator_cmd_vel.
"""

import math
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String


class PhysicsTestGate(Node):
    def __init__(self):
        super().__init__('sim_physics_test_gate')
        self.mode = 'disabled'
        self.command = None
        self.command_at = 0.0
        self.output = Twist()
        self.last = time.monotonic()
        self.healthy_at = {}
        self.upright = False
        self.contact_at = -1e9
        self.posture_ready = None
        self.posture_at = 0.0
        self.pub = self.create_publisher(Twist, '/sim/actuator_cmd_vel', 10)
        self.active_pub = self.create_publisher(Bool, '/sim/physics_test_active', 10)
        self.status = self.create_publisher(String, '/sim/physics_test_status', 10)
        self.create_subscription(
            Odometry, '/sim/ground_truth_odometry', self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', lambda m: self.on_sensor('imu'),
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', lambda m: self.on_sensor('joints'),
                                 qos_profile_sensor_data)
        self.create_subscription(Contacts, '/assessment/body_contacts', self.on_contact,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/sim/posture_ready', self.on_posture_ready, 10)
        self.create_subscription(String, '/sim/control_mode', self.on_mode, 10)
        self.create_subscription(Twist, '/sim/manual_cmd_vel', self.on_cmd, 10)
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.active_pub.publish(Bool(data=True))
        self.get_logger().warning(
            'DIAGNOSTIC physics-test gate: ground truth is not localization.')

    def on_sensor(self, name):
        self.healthy_at[name] = time.monotonic()

    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        # Reject excessive roll/pitch; the test only ever runs level.
        self.upright = (math.isfinite(norm) and abs(norm - 1.0) < 0.01 and
                        1.0 - 2.0 * (q.x * q.x + q.y * q.y) > math.cos(0.35))
        self.on_sensor('truth')
        if not self.upright:
            self.command = None

    def on_contact(self, msg):
        if not msg.contacts:
            return
        self.contact_at = time.monotonic()
        self.command = None

    def on_posture_ready(self, msg):
        self.posture_ready = msg.data
        self.posture_at = time.monotonic()

    def on_mode(self, msg):
        if msg.data not in ('disabled', 'manual', 'estop'):
            return
        if msg.data != self.mode:
            self.mode = msg.data
            self.command = None
            self.output = Twist()
            self.pub.publish(self.output)

    def on_cmd(self, msg):
        if self.mode != 'manual':
            return
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not all(math.isfinite(x) for x in values):
            self.command = None
            return
        self.command = values
        self.command_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.05)
        self.last = now
        reason = 'active'
        if self.mode not in ('manual',):
            reason = self.mode
        elif any(now - self.healthy_at.get(k, 0.0) > 0.5 for k in ('truth', 'imu', 'joints')):
            reason = 'diagnostic_feedback_stale'
        elif not self.upright:
            reason = 'tilt'
        elif self.posture_ready is None or now - self.posture_at > 0.5:
            reason = 'posture_feedback_unavailable'
        elif not self.posture_ready:
            reason = 'posture_not_ready'
        elif now - self.contact_at < 0.35:
            reason = 'body_contact_stop'
        elif self.command is None or now - self.command_at > 0.3:
            reason = 'command_timeout'
        if reason != 'active':
            self.command = None
            self.output = Twist()
        else:
            x, y, w = self.command
            norm = math.hypot(x, y)
            if norm > 0.3:
                x *= 0.3 / norm
                y *= 0.3 / norm
            w = max(-0.5, min(0.5, w))
            old = [self.output.linear.x, self.output.linear.y, self.output.angular.z]
            targets = [x, y, w]
            step = [0.3 * dt, 0.3 * dt, 0.8 * dt]
            new = [a + max(-s, min(s, b - a)) for a, b, s in zip(old, targets, step)]
            self.output.linear.x, self.output.linear.y, self.output.angular.z = new
        self.pub.publish(self.output)
        self.status.publish(String(data='diagnostic:' + reason))
        # Keep the diagnostic flag observable for late joiners; the one-shot
        # startup publication is invisible to subscribers created after launch.
        self.active_pub.publish(Bool(data=True))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PhysicsTestGate()
    stopping = False
    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.active_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
