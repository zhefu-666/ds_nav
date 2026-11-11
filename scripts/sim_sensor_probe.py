#!/usr/bin/env python3
"""Read-only simulation input/estimator audit, with bounded wall-clock duration."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from livox_ros_driver2.msg import CustomMsg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('sim_sensor_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    counts, latest, stamps = {}, {}, {}

    def receive(key, convert):
        def callback(msg):
            counts[key] = counts.get(key, 0) + 1
            latest[key] = convert(msg)
            if hasattr(msg, 'header'):
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                stamps.setdefault(key, []).append(stamp)
        return callback

    def pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        return {'frame': msg.header.frame_id, 'child': msg.child_frame_id,
                'xyz': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w],
                'v': [v.x, v.y, v.z], 'w': [w.x, w.y, w.z]}

    def cloud(msg):
        data = point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False)
        xyz = np.column_stack([data[k] for k in ['x','y','z']])
        finite = np.isfinite(xyz).all(axis=1)
        return {'frame': msg.header.frame_id, 'slots': len(xyz), 'finite': int(finite.sum()),
                'near_0_5m': int((np.linalg.norm(xyz[finite], axis=1)<.5).sum())}

    subscriptions = [
        (Odometry, '/aft_mapped_to_init', 'lio', pose),
        (Odometry, '/sim/base_odom', 'base', pose),
        (Odometry, '/sim/lidar_odom', 'lidar_odom', pose),
        (Odometry, '/sim/ground_truth_odometry', 'truth_diagnostic', pose),
        (Imu, '/livox/imu', 'imu', lambda m: {'frame':m.header.frame_id,
            'acc':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],
            'gyro':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]}),
        (JointState, '/joint_states', 'joints', lambda m: dict(zip(m.name,m.position))),
        (PointCloud2, '/lidar/points', 'raw_cloud', cloud),
        (CustomMsg, '/livox/lidar', 'livox', lambda m: {'points':m.point_num,
            'offset_ns':sorted(set(p.offset_time for p in m.points)),
            'finite':all(math.isfinite(p.x+p.y+p.z) for p in m.points)}),
    ]
    for typ, topic, key, convert in subscriptions:
        node.create_subscription(typ, topic, receive(key, convert), qos_profile_sensor_data)
    for typ, topic, key in [(Bool,'/sim/localization_valid','valid'),
                           (String,'/sim/localization_status','localization_status'),
                           (String,'/gicp_relocalization_node/localization_state','gicp'),
                           (String,'/sim/control_status','control_status')]:
        node.create_subscription(typ, topic, receive(key, lambda m:m.data), 10)
    start = time.monotonic()
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        result = {'diagnostic_only':True, 'wall_seconds':time.monotonic()-start,
                  'counts':counts, 'latest':latest,
                  'measurement_intervals':{k:{'span':v[-1]-v[0],
                    'nonmonotonic':sum(b<a for a,b in zip(v,v[1:])),
                    'unique_stamps':len(set(v))} for k,v in stamps.items() if v}}
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
