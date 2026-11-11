#!/usr/bin/env python3
"""Articulated posture controller for the ds_nav simulation.

Owns posture joints only. It never publishes chassis velocity, odometry or TF;
the single actuator owner stays sim_command_gate.py (or the physics-test gate).
Joint targets go to the Gazebo JointPositionController topics that bridge.yaml
already routes. Readiness is decided from actual /joint_states feedback, not
from the fact that a posture string was sent.
"""

import json
import math
from pathlib import Path
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

_SHARE = None


def _import_kinematic_math():
    global _SHARE
    share = Path(get_package_share_directory('huananhu_sim_resources'))
    _SHARE = share
    scripts = str(share / 'scripts')
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    from kinematic_math import joint_positions, smooth_step

    return joint_positions, smooth_step


# Revolute joints that define the visible posture. Gas slides are prismatic and
# are compared in metres, so they are excluded from the angle tolerance check.
POSTURE_JOINTS = (
    'yaw1_joint', 'yaw2_joint',
    'pitch1_drive_joint', 'pitch1_rocker_joint', 'pitch1_coupler_joint',
    'uwb_level_joint', 'pitch2_joint',
    'yaw1_motor_joint', 'yaw2_motor_joint',
    'gas_1_hinge', 'gas_2_hinge',
)


class PostureController(Node):
    def __init__(self, joint_positions, smooth_step):
        super().__init__('sim_posture_controller')
        self.joint_positions = joint_positions
        self.smooth_step = smooth_step
        self.config = json.loads((_SHARE / 'config' / 'model.json').read_text(encoding='utf-8'))
        self.tunnel_profile = json.loads((_SHARE / 'config' / 'tunnel_profile.json').read_text())
        self.tolerance = float(self.tunnel_profile.get('joint_tolerance_rad', 0.01))
        self.tunnel_posture = True
        # Posture only: base motion, odom and TF belong to the gate/adapter chain.
        self.declare_parameter('publish_base_motion', False)
        self.declare_parameter('feedback_stale_after', 0.5)
        self.declare_parameter('ready_hold_time', 0.3)
        if bool(self.get_parameter('publish_base_motion').value):
            raise ValueError('sim_posture_controller must not own base motion')
        self.feedback_stale_after = float(self.get_parameter('feedback_stale_after').value)
        self.ready_hold_time = float(self.get_parameter('ready_hold_time').value)

        self.positions = {name: 0.0 for name in self.config['controlled_joints']}
        # Wheels belong to the traction plugin, never to posture position targets.
        wheel_joints = {wheel['joint'] for wheel in self.config['wheels']}
        self.joint_publishers = {
            name: self.create_publisher(Float64, '/huananhu/position/' + name, 10)
            for name in self.config['controlled_joints']
            if name not in wheel_joints
        }
        self.state_pub = self.create_publisher(String, '/sim/posture_state', 10)
        self.ready_pub = self.create_publisher(Bool, '/sim/posture_ready', 10)
        self.create_subscription(String, '/huananhu/posture', self.on_posture, 10)
        self.create_subscription(Float64MultiArray, '/huananhu/yaw', self.on_yaw, 10)
        self.create_subscription(Float64, '/huananhu/aim', self.on_aim, 10)
        self.actual_joints = {}
        self.actual_stamp = None
        self.contact_hold = None
        self.last_contact = -100.0
        self.create_subscription(
            JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(
            Contacts, '/assessment/body_contacts', self.on_contact, qos_profile_sensor_data)

        self.start = self.last = self.get_clock().now().nanoseconds / 1e9
        self.last_posture = -100.0
        self.target_fold = 0.0
        self.target_yaw1 = self.target_yaw2 = 0.0
        self.target_aim = self.tunnel_profile['aim_rad']
        self.fold = self.yaw1 = self.yaw2 = self.aim = 0.0
        self.ready_since = None
        self.timer = self.create_timer(1 / 30.0, self.update)
        self.get_logger().info(
            'Posture-only controller: /joint_states feedback, no base motion/TF.')

    def on_joints(self, msg):
        self.actual_joints = dict(zip(msg.name, msg.position))
        self.actual_stamp = self.get_clock().now()

    def on_contact(self, msg):
        if not msg.contacts or not self.actual_joints:
            return
        self.last_contact = self.get_clock().now().nanoseconds / 1e9
        if self.contact_hold is None:
            self.contact_hold = dict(self.actual_joints)

    def on_posture(self, msg):
        if msg.data in ('stand', '起身', '正常'):
            self.target_fold = math.radians(81)
            self.tunnel_posture = False
            self.target_aim = 0.0
        elif msg.data in ('tunnel', '卧倒', '过洞'):
            self.target_fold = self.tunnel_profile['fold_rad']
            self.tunnel_posture = True
        else:
            self.get_logger().warning('姿态只接受 stand 或 tunnel。')
            return
        self.last_posture = self.get_clock().now().nanoseconds / 1e9

    def on_yaw(self, msg):
        if len(msg.data) >= 2:
            self.target_yaw1 = max(-math.pi, min(math.pi, msg.data[0]))
            self.target_yaw2 = max(-math.pi / 2, min(math.pi / 2, msg.data[1]))

    def on_aim(self, msg):
        self.target_aim = max(math.radians(-20), min(math.radians(36), msg.data))

    def _feedback_fresh(self, now):
        if self.actual_stamp is None:
            return False
        return (now - self.actual_stamp.nanoseconds * 1e-9) <= self.feedback_stale_after

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last
        if dt <= 0 or dt > 0.25:
            self.last = now
            return
        self.last = now
        target_fold, y1, y2, aim = (
            self.target_fold, self.target_yaw1, self.target_yaw2, self.target_aim)
        if target_fold == self.tunnel_profile['fold_rad'] and self.tunnel_posture:
            # Tunnel posture coordinates the two yaw axes and head pitch.
            # External aim=0 must not silently undo the validated low profile.
            y1 = self.tunnel_profile['yaw1_rad']
            y2 = self.tunnel_profile['yaw2_rad']
            aim = self.tunnel_profile['aim_rad']
        self.fold = self.smooth_step(self.fold, target_fold, 0.45, dt)
        self.yaw1 = self.smooth_step(self.yaw1, y1, 0.7, dt)
        self.yaw2 = self.smooth_step(self.yaw2, y2, 0.9, dt)
        self.aim = self.smooth_step(self.aim, aim, 0.5, dt)
        q = self.joint_positions(
            self.config, self.fold, self.yaw1, self.yaw2, self.aim, self.positions)
        if self.contact_hold is not None:
            if now - self.last_contact < 0.35:
                # Do not keep forcing position-controlled mechanisms through an obstacle.
                for name in q:
                    if not name.startswith('wheel_') and name in self.contact_hold:
                        q[name] = self.contact_hold[name]
                self.fold = self.contact_hold.get('pitch1_drive_joint', self.fold)
            else:
                self.contact_hold = None
        for name, pub in self.joint_publishers.items():
            msg = Float64()
            msg.data = float(q[name])
            pub.publish(msg)
        self.publish_readiness(q, now)

    def publish_readiness(self, q, now):
        state = 'tunnel' if self.tunnel_posture else 'stand'
        ready = False
        if self._feedback_fresh(now):
            ready = all(
                abs(self.actual_joints.get(name, 1e9) - q[name]) <= self.tolerance
                for name in POSTURE_JOINTS
                if name in q)
        if ready:
            if self.ready_since is None:
                self.ready_since = now
            ready = now - self.ready_since >= self.ready_hold_time
        else:
            self.ready_since = None
        self.state_pub.publish(String(data=state if ready else 'transition:' + state))
        self.ready_pub.publish(Bool(data=ready))


def main():
    rclpy.init()
    try:
        joint_positions, smooth_step = _import_kinematic_math()
        node = PostureController(joint_positions, smooth_step)
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
