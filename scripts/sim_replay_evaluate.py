#!/usr/bin/env python3
"""Read CLOSED bags; recover measurement-time base pose and reuse official scoring.
For CAD yaw1 co-located sensors only (not low_profile).
Caller must stop recording first; metadata alone cannot prove writer absence.
No ROS node, publications, controller input or changes to historical results.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/simulation_integration/huananhu_sim_bridge/scripts'))
from sensor_state_math import JointHistory, mount_state, recover
from sim_trajectory_evaluate import evaluate


def bag_messages(directory, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    directory = Path(directory)
    if not (directory / 'metadata.yaml').is_file():
        raise ValueError('closed bag with metadata.yaml required')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in topics:
        if topic not in types:
            raise ValueError('missing bag topic: '+topic)
    classes = {t: get_message(types[t]) for t in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in classes:
            yield topic, deserialize_message(data, classes[topic])


def pose_row(msg, kind):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return {'kind': kind, 'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            'parent_frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
            'pose': {'position': [p.x,p.y,p.z], 'orientation': [q.x,q.y,q.z,q.w]}}


def score_rows(estimates, truth, joints):
    """Preserve acquisition order: rollback is an error, never sort it away."""
    history = JointHistory(max_gap=.1)
    previous = None
    for stamp, angle in joints:
        if not math.isfinite(angle) or stamp <= 0:
            raise ValueError('invalid_joint_measurement')
        if previous is not None and stamp <= previous:
            raise ValueError('non_monotonic_joint_time')
        history.samples.append((stamp*1e-9, angle))
        previous = stamp
    rejected = Counter()
    events = [{'kind': 'metadata', 'mode': 'mapping'}]+truth
    for row in estimates:
        # Keep rejected rows in the denominator; never silently improve coverage.
        out = dict(row, kind='estimate', child_frame='base_footprint')
        try:
            if row['parent_frame'] != 'camera_init' or row['child_frame'] != 'body':
                raise ValueError('unexpected_sensor_odometry_frame')
            p = row['pose']
            q = p['orientation']
            if (not all(math.isfinite(v) for v in (*p['position'], *q)) or
                    abs(sum(v*v for v in q)-1.) > .01):
                raise ValueError('invalid_sensor_pose')
            angle, rate = history.at(row['stamp_ns']*1e-9)
            position, orientation, _, _ = recover(p['position'], q, (0,0,0), (0,0,0),
                                                 mount_state(angle, rate))
            out['pose'] = {'position': list(position), 'orientation': list(orientation)}
        except ValueError as exc:
            rejected[str(exc)] += 1
            out['child_frame'] = 'INVALID_RECOVERY'
        events.append(out)
    result = evaluate(events)
    result.update(tool='sim_replay_evaluate_v1', recovery_rejected=dict(rejected),
                  reference='measurement-time yaw1 recovery; chassis local-z -0.10m',
                  source_kind='offline_replay', truth_used_for_control=False,
                  sensor_contract='huananhu_yaw1_colocated_body_origin_v2',
                  chassis_to_base_local_xyz=[0.,0.,-.10],
                  contact_evidence_available=False,
                  evaluation_scope='pose only; no collision or navigation acceptance claim')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-bag', type=Path, required=True)
    parser.add_argument('--odom-bag', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error('output exists; preserve historical results')
    truth, joints = [], []
    try:
        for topic, msg in bag_messages(args.raw_bag, ['/sim/ground_truth_odometry', '/joint_states']):
            if topic == '/joint_states':
                if 'yaw1_joint' in msg.name:
                    joints.append((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                                   msg.position[msg.name.index('yaw1_joint')]))
            else:
                truth.append(pose_row(msg, 'truth'))
        est = [pose_row(msg, 'estimate') for _, msg in
               bag_messages(args.odom_bag, ['/aft_mapped_to_init'])]
        result = score_rows(est, truth, joints)
    except (ValueError, RuntimeError, IndexError) as exc:
        result = {'status': 'FAIL', 'reasons': [str(exc)], 'truth_used_for_control': False}
    result['inputs'] = {'raw_bag': str(args.raw_bag.resolve()),
                        'odom_bag': str(args.odom_bag.resolve())}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as stream:
        stream.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'SCORED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
