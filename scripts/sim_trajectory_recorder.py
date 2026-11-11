#!/usr/bin/env python3
"""Bounded read-only evidence recorder. Truth is never published or used by control.

Run in the same ROS domain as a trial, e.g. --mode prior --duration 180 --out
artifacts/.../trajectory.jsonl. Evaluate after trial with sim_trajectory_evaluate.py.
Duration is wall time so paused simulation cannot leave this process unbounded.
"""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time
import signal

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from ros_interfaces.msg import MpcPositionCommand
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

from sim_trajectory_evaluate import compose


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def pose_dict(pose):
    p, q = pose.position, pose.orientation
    return {'position': [p.x,p.y,p.z], 'orientation': [q.x,q.y,q.z,q.w]}


class Recorder(Node):
    def __init__(self, stream, mode, tf_wait):
        super().__init__('sim_trajectory_recorder',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.stream, self.mode, self.tf_wait = stream, mode, tf_wait
        self.counts, self.pending = Counter(), deque()
        self.buffer = Buffer(cache_time=Duration(seconds=120))
        self.listener = TransformListener(self.buffer, self)
        self.write({'kind':'metadata', 'version':1, 'mode':mode,
                    'duration_clock':'wall', 'pose_stamp_clock':'simulation',
                    'truth_topic':'/sim/ground_truth_odometry', 'truth_pose_origin':'chassis',
                    'truth_to_base_translation_chassis_m':[0.,0.,-.10],
                    'estimate_topic':'/sim/base_odom', 'estimate_pose_origin':'base_footprint',
                    'prior_frame_contract':'map equals world; TF queried at each estimate measurement stamp',
                    'mapping_frame_contract':'camera_init local gauge; fixed initial SE3 alignment offline only',
                    'saved_local_frame_contract':'estimate transformed into saved-local map at stamp; fixed initial SE3 alignment offline only',
                    'truth_used_for_control':False, 'tf_wait_wall_s':tf_wait})
        self.create_subscription(Odometry, '/sim/ground_truth_odometry', self.truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/base_odom', self.estimate, qos_profile_sensor_data)
        self.create_subscription(CustomMsg, '/livox/lidar', self.lidar, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(Twist, '/sim/actuator_cmd_vel', self.command, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/assessment/body_contacts', self.contact, qos_profile_sensor_data)
        self.create_subscription(MpcPositionCommand, '/opt_path', self.opt_path, qos_profile_sensor_data)

    def write(self, row):
        row.setdefault('receipt_wall_monotonic_s', time.monotonic())
        self.stream.write(json.dumps(row, allow_nan=False)+'\n')
        self.counts[row['kind']] += 1
        if sum(self.counts.values()) % 100 == 0:
            self.stream.flush()

    def truth(self, msg):
        self.write({'kind':'truth', 'stamp_ns':stamp_ns(msg.header.stamp),
                    'parent_frame':msg.header.frame_id, 'child_frame':msg.child_frame_id,
                    'pose':pose_dict(msg.pose.pose)})

    def estimate(self, msg):
        row = {'kind':'estimate', 'stamp_ns':stamp_ns(msg.header.stamp),
               'parent_frame':msg.header.frame_id, 'child_frame':msg.child_frame_id,
               'receipt_wall_monotonic_s':time.monotonic(),
               'pose':pose_dict(msg.pose.pose),
               'twist_linear':[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]}
        self.pending.append((time.monotonic(), row))
        self.flush_pending()

    def flush_pending(self, force=False):
        # FIFO preserves estimator ordering, including detectable clock rollback.
        while self.pending:
            received, row = self.pending[0]
            if row['stamp_ns'] <= 0:
                row['tf_status'] = 'INVALID_ZERO_MEASUREMENT_STAMP'
            elif self.mode == 'mapping':
                row['tf_status'] = 'LOCAL_GAUGE_NO_GLOBAL_TF_REQUIRED'
            else:
                try:
                    tf = self.buffer.lookup_transform('map', row['parent_frame'],
                                                      Time(nanoseconds=row['stamp_ns']))
                    tr = tf.transform
                    transform = {'position':[tr.translation.x,tr.translation.y,tr.translation.z],
                                 'orientation':[tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]}
                    row.update(tf_status='OK', map_tf_query_stamp_ns=row['stamp_ns'],
                               map_tf_header_stamp_ns=stamp_ns(tf.header.stamp),
                               transform_map_from_estimate_frame=transform,
                               pose_map=compose(transform, row['pose']))
                except Exception as exc:
                    if not force and time.monotonic()-received < self.tf_wait:
                        break
                    row.update(tf_status='UNAVAILABLE_AT_MEASUREMENT_STAMP', tf_error=str(exc))
            self.pending.popleft()
            self.write(row)

    def lidar(self, msg):
        offsets = [p.offset_time for p in msg.points]
        self.write({'kind':'sensor_lidar', 'stamp_ns':stamp_ns(msg.header.stamp),
                    'frame':msg.header.frame_id, 'timebase_ns':int(msg.timebase),
                    'point_count':int(msg.point_num),
                    'offset_ns_min':min(offsets) if offsets else None,
                    'offset_ns_max':max(offsets) if offsets else None})

    def imu(self, msg):
        self.write({'kind':'sensor_imu', 'stamp_ns':stamp_ns(msg.header.stamp),
                    'frame':msg.header.frame_id,
                    'linear_acceleration_m_s2':[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                    'angular_velocity_rad_s':[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                    'orientation_xyzw':[msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                    'orientation_covariance':list(msg.orientation_covariance)})

    def command(self, msg):
        self.write({'kind':'command', 'stamp_ns':self.get_clock().now().nanoseconds,
                    'stamp_source':'receipt_simulation_clock_unstamped_twist',
                    'topic':'/sim/actuator_cmd_vel',
                    'velocity':[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]})

    def contact(self, msg):
        self.write({'kind':'contact', 'stamp_ns':stamp_ns(msg.header.stamp),
                    'contacts':len(msg.contacts),
                    'pairs':[[c.collision1.name,c.collision2.name] for c in msg.contacts]})

    def opt_path(self, msg):
        self.write({'kind':'opt_path', 'stamp_ns':stamp_ns(msg.header.stamp),
                    'frame':msg.header.frame_id, 'command_count':len(msg.cmds),
                    'command_flag':int(msg.command_flag), 'mpc_horizon':int(msg.mpc_horizon)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['prior','mapping','saved-local'], required=True)
    parser.add_argument('--duration', type=float, required=True, help='maximum wall seconds')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--tf-wait', type=float, default=2., help='wall seconds to await exact-stamp TF')
    args = parser.parse_args()
    if not 0 < args.duration <= 86400 or not 0 < args.tf_wait <= 30:
        parser.error('duration in (0,86400], tf-wait in (0,30] required')
    if args.out.exists(): parser.error('output exists; choose a new path')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    def terminate(_signal, _frame):
        raise KeyboardInterrupt
    rclpy.init()
    signal.signal(signal.SIGTERM, terminate)
    with args.out.open('x') as stream:
        node = Recorder(stream, args.mode, args.tf_wait)
        started = time.monotonic()
        reason = 'duration_reached'
        try:
            while rclpy.ok() and time.monotonic()-started < args.duration:
                rclpy.spin_once(node, timeout_sec=.05)
                node.flush_pending()
        except (KeyboardInterrupt, ExternalShutdownException):
            reason = 'interrupted'
        finally:
            node.flush_pending(force=True)
            node.write({'kind':'footer', 'reason':reason,
                        'wall_duration_s':time.monotonic()-started, 'counts':dict(node.counts)})
            node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    print(str(args.out))


if __name__ == '__main__':
    main()
