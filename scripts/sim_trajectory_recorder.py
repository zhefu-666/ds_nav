#!/usr/bin/env python3
"""Bounded read-only evidence recorder. Truth is never published or used by control.

Run in the same ROS domain as a trial, e.g. --mode prior --duration 180 --out
artifacts/.../trajectory.jsonl. Evaluate after trial with sim_trajectory_evaluate.py.
Duration is wall time so paused simulation cannot leave this process unbounded.
"""
import argparse
from collections import Counter, deque
from contextlib import ExitStack
import json
from pathlib import Path
import time
import signal

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from ros_interfaces.msg import MpcPositionCommand
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
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
        model = json.loads((Path(get_package_share_directory('huananhu_sim_resources')) / 'config/model.json').read_text())
        wheel_joints = {wheel['joint'] for wheel in model['wheels']}
        self.recorded_joints = {joint['name'] for joint in model['joints']
                                if joint['type'] != 'fixed' and joint['name'] not in wheel_joints}
        self.counts, self.pending = Counter(), deque()
        self.last_receipts = {}
        self.contact_complete = False
        self.buffer = Buffer(cache_time=Duration(seconds=120))
        self.listener = TransformListener(self.buffer, self)
        self.metadata = {'kind':'metadata', 'version':1, 'mode':mode,
                    'duration_clock':'wall', 'pose_stamp_clock':'simulation',
                    'truth_topic':'/sim/ground_truth_odometry', 'truth_pose_origin':'chassis',
                    'truth_to_base_translation_chassis_m':[0.,0.,-.10],
                    'estimate_topic':'/sim/base_odom', 'estimate_pose_origin':'base_footprint',
                    'prior_frame_contract':'map equals world; TF queried at each estimate measurement stamp',
                    'mapping_frame_contract':'camera_init local gauge; fixed initial SE3 alignment offline only',
                    'saved_local_frame_contract':'estimate transformed into saved-local map at stamp; fixed initial SE3 alignment offline only',
                    'truth_used_for_control':False, 'tf_wait_wall_s':tf_wait}
        self.write(self.metadata.copy())
        self.create_subscription(Odometry, '/sim/ground_truth_odometry', self.truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/base_odom', self.estimate, qos_profile_sensor_data)
        self.create_subscription(CustomMsg, '/livox/lidar', self.lidar, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(Twist, '/sim/actuator_cmd_vel', self.command, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/assessment/body_contacts', self.contact, qos_profile_sensor_data)
        self.create_subscription(String, '/assessment/body_contact_status', self.contact_status, qos_profile_sensor_data)
        self.create_subscription(MpcPositionCommand, '/opt_path', self.opt_path, qos_profile_sensor_data)

    def write(self, row):
        row.setdefault('receipt_wall_monotonic_s', time.monotonic())
        self.stream.write(json.dumps(row, allow_nan=False)+'\n')
        self.counts[row['kind']] += 1
        self.last_receipts[row['kind']] = row['receipt_wall_monotonic_s']
        if row['kind'] == 'contact_status':
            self.contact_complete = row.get('complete') is True
        if sum(self.counts.values()) % 100 == 0:
            self.stream.flush()

    def capture_ready(self, now):
        kinds = ('truth', 'command', 'contact_status', 'joint')
        return self.contact_complete and all(self.counts[kind] >= 2 and
            0 <= now-self.last_receipts.get(kind, -1e9) <= .5 for kind in kinds)

    def start_capture(self, stream, preflight_path):
        self.flush_pending(force=True)
        self.write({'kind':'footer', 'reason':'preflight_complete', 'counts':dict(self.counts)})
        self.stream.flush()
        self.stream = stream
        self.counts = Counter()
        header = dict(self.metadata, preflight_file=preflight_path,
                      capture_protocol='fresh_streams_before_mission')
        self.write(header)
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

    def joints(self, msg):
        self.write({'kind':'joint', 'stamp_ns':stamp_ns(msg.header.stamp),
                    'joints':{name:value for name,value in zip(msg.name, msg.position)
                              if name in self.recorded_joints}})

    def contact_status(self, msg):
        payload = json.loads(msg.data)
        self.write({'kind':'contact_status', 'stamp_ns':payload['stamp_ns'],
                    'contacts':payload['contacts'], 'complete':payload['complete'],
                    'sensors':payload['sensors'], 'expected_sensors':payload['expected_sensors']})

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
    parser.add_argument('--warmup-seconds', type=float, default=0., help='retain preflight separately before formal capture')
    parser.add_argument('--ready-file', type=Path, help='exclusive readiness sidecar for the owning fixture')
    args = parser.parse_args()
    if not 0 < args.duration <= 86400 or not 0 < args.tf_wait <= 30:
        parser.error('duration in (0,86400], tf-wait in (0,30] required')
    if not 0 <= args.warmup_seconds <= 10:
        parser.error('warmup-seconds in [0,10] required')
    preflight = args.out.with_name(args.out.stem+'.preflight.jsonl')
    for path in [args.out, args.ready_file, preflight if args.warmup_seconds else None]:
        if path is not None and path.exists():parser.error('output exists; choose a new path')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    stopping = False
    def terminate(_signal, _frame):
        nonlocal stopping
        stopping = True
    # Do not raise asynchronously inside rclpy's C++ message conversion. Finish
    # the current bounded spin, then write the footer and shut down normally.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    with ExitStack() as stack:
        stream = stack.enter_context((preflight if args.warmup_seconds else args.out).open('x'))
        node = Recorder(stream, args.mode, args.tf_wait)
        capturing = not args.warmup_seconds
        started = time.monotonic()
        reason = 'duration_reached'
        try:
            while not stopping and rclpy.ok() and time.monotonic()-started < args.duration:
                rclpy.spin_once(node, timeout_sec=.05)
                node.flush_pending()
                if not capturing:
                    elapsed = time.monotonic()-started
                    if elapsed >= args.warmup_seconds and node.capture_ready(time.monotonic()):
                        node.start_capture(stack.enter_context(args.out.open('x')), str(preflight.resolve()))
                        capturing = True
                        if args.ready_file:
                            with args.ready_file.open('x') as ready:
                                json.dump({'ready':True,'preflight_file':str(preflight.resolve())},ready)
                    elif elapsed > 30:
                        raise RuntimeError('recording_streams_not_ready')
            if stopping:
                reason = 'interrupted'
        except (KeyboardInterrupt, ExternalShutdownException):
            reason = 'interrupted'
        except Exception:
            reason = 'error'
            raise
        finally:
            node.flush_pending(force=True)
            node.write({'kind':'footer', 'reason':reason,
                        'wall_duration_s':time.monotonic()-started, 'counts':dict(node.counts)})
            node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    print(str(args.out))


if __name__ == '__main__':
    main()
