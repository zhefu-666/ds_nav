#!/usr/bin/env python3
"""Convert Gazebo acquisition frames into Livox messages.

snapshot preserves the legacy instantaneous frame. Experimental micro_scan
selects one azimuth sector from each of four independent 40 Hz acquisitions,
retaining the measured coordinates and actual timestamps, then publishes a
10 Hz scan. No truth-based compensation or invented per-ray times are used.
The four batches have 25 ms time resolution; this is not a vendor MID360 model.
Motion distortion, moving occlusion and yaw1 installation still need acceptance.
"""

import copy
import time
from collections import deque
from cad_self_filter import CadSelfFilter, MeasuredJointHistory

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2 as pc2

from livox_ros_driver2.msg import CustomMsg, CustomPoint


class LidarToLivox(Node):
    def __init__(self):
        super().__init__("sim_lidar_to_livox")
        self.declare_parameter("input_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("scan_lines", 32)
        self.declare_parameter("vertical_min_angle", -0.12217304764)
        self.declare_parameter("vertical_max_angle", 0.90757121104)

        self.declare_parameter("sampling_mode", "snapshot")
        self.declare_parameter("micro_batches", 4)
        self.declare_parameter("micro_rate_hz", 40.0)
        # E2 diagnostic/candidate (20261030 GLM run): micro composites failed
        # Point-LIO tracking two independent ways: 4 sector-wedge EKF batches
        # (bin=2ms) lose +y translation; one whole-FOV batch (bin=1000) tracks
        # in-motion xy but drifts/z-collapses without the gutted undistortion
        # (IMU_Processing.cpp Process copies the cloud unchanged). wedge40
        # publishes each 40 Hz acquisition's sector as an instantaneous scan at
        # its own true stamp: no cross-acquisition composition, no synthetic
        # per-point offsets, real 40 Hz independent acquisitions preserved.
        self.declare_parameter("micro_publish_mode", "composite")
        self.micro_publish_mode = str(self.get_parameter("micro_publish_mode").value)
        self.mode = self.get_parameter("sampling_mode").value
        self.batch_count = int(self.get_parameter("micro_batches").value)
        self.batch_period_ns = int(1e9 / float(self.get_parameter("micro_rate_hz").value))
        if self.mode not in ("snapshot", "micro_scan") or self.batch_count < 2:
            raise ValueError("Unsupported sampling mode or micro batch count")
        if self.micro_publish_mode not in ("composite", "wedge40"):
            raise ValueError("Unsupported micro publish mode")
        self.declare_parameter("self_filter", False)
        self.declare_parameter("self_filter_resource_dir", "")
        self.declare_parameter("self_filter_tolerance", .001)
        self.declare_parameter("self_filter_geometry", "visual_surface")
        self.declare_parameter("self_filter_joint_topic", "/joint_states")
        self.declare_parameter("self_filter_max_joint_gap", .1)
        self.self_filter = None
        self.cloud_queue = deque()
        self.self_counts = dict(batches=0, removed=0, dropped=0, history_errors=0)
        self.last_self_log = 0.0
        if self.get_parameter("self_filter").value:
            self.self_filter = CadSelfFilter(
                self.get_parameter("self_filter_resource_dir").value,
                float(self.get_parameter("self_filter_tolerance").value),
                self.get_parameter("self_filter_geometry").value)
            self.joint_history = MeasuredJointHistory(self.self_filter.required,
                float(self.get_parameter("self_filter_max_joint_gap").value))
            self.create_subscription(JointState,
                self.get_parameter("self_filter_joint_topic").value,
                self.on_joint, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=100))
        self.pending = []
        self.batch_index = 0
        self.first_header = None
        self.last_ns = None

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.scan_lines = int(self.get_parameter("scan_lines").value)
        self.v_min = float(self.get_parameter("vertical_min_angle").value)
        self.v_max = float(self.get_parameter("vertical_max_angle").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(CustomMsg, out_topic, qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.on_cloud, qos)
        self.get_logger().info(f"{in_topic} -> {out_topic} ({self.mode})")

    def reset_scan(self):
        self.pending = []
        self.batch_index = 0
        self.last_ns = None

    def self_log(self):
        now = time.monotonic()
        if now-self.last_self_log >= 5.0:
            self.last_self_log = now
            self.get_logger().info(f"CAD self-filter counters {self.self_counts}")

    def on_joint(self, msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.joint_history.append(stamp, msg.name, msg.position)
        except ValueError:
            self.self_counts['history_errors'] += 1
            self.self_counts['dropped'] += len(self.cloud_queue)
            self.cloud_queue.clear()
            self.reset_scan()
            self.self_log()
            return
        self.drain_clouds(stamp)

    def drain_clouds(self, stamp):
        while self.cloud_queue:
            cloud = self.cloud_queue[0]
            cloud_stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
            if cloud_stamp >= stamp:
                break
            self.cloud_queue.popleft()
            try:
                positions = self.joint_history.at(cloud_stamp)
                self.process_cloud(cloud, positions)
            except ValueError:
                self.self_counts['dropped'] += 1
                self.reset_scan()
            self.self_log()

    def on_cloud(self, msg: PointCloud2):
        if self.self_filter is None:
            self.process_cloud(msg)
            return
        self.cloud_queue.append(msg)
        if len(self.cloud_queue) > 8:
            self.cloud_queue.popleft()
            self.self_counts['dropped'] += 1
            self.reset_scan()
            self.self_log()
        # Joint messages can already bracket this acquisition when its DDS
        # callback runs. Do not leave it waiting for another joint callback,
        # whose queue may itself be delayed by the preceding geometry work.
        if self.joint_history.samples:
            self.drain_clouds(self.joint_history.samples[-1][0])

    def process_cloud(self, msg: PointCloud2, positions=None):
        fields = {f.name for f in msg.fields}
        names = ["x", "y", "z", "intensity"]
        has_ring = "ring" in fields
        if has_ring:
            names.append("ring")
        points = pc2.read_points(msg, field_names=names, skip_nans=False)
        if len(points) == 0:
            return
        # read_points returns a structured array; column_stack keeps this
        # working for both structured and tuple-list implementations.
        data = np.column_stack(
            [np.asarray(points[name], dtype=np.float64) for name in names]
        )
        # Fortress represents missed rays as infinities, not NaNs. They are
        # absent returns, not Livox measurements, and must not reach the LIO.
        # IDs refer to flattened ORIGINAL PointCloud slots, before removal of
        # nonfinite returns or sector selection. They never encode hit masks.
        ray_ids=np.arange(len(data),dtype=np.int64)
        finite=np.isfinite(data[:, :3]).all(axis=1)
        data,ray_ids=data[finite],ray_ids[finite]
        # Select the actual acquisition sector BEFORE expensive geometry work.
        # This preserves the existing per-batch stamp and ring derivation.
        if self.mode == "micro_scan":
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if self.last_ns is not None and (stamp_ns <= self.last_ns or
                    stamp_ns-self.last_ns > self.batch_period_ns*1.5):
                self.reset_scan()
            azimuth = np.arctan2(data[:,1],data[:,0])
            sector = np.clip(np.floor((azimuth+np.pi)/(2*np.pi)*self.batch_count),0,self.batch_count-1).astype(int)
            selected=sector == self.batch_index
            data,ray_ids=data[selected],ray_ids[selected]
        if self.self_filter is not None:
            if msg.header.frame_id != "navigation_laser":
                raise ValueError("unexpected_lidar_measurement_frame")
            hits, _ = self.self_filter.classify(data[:, :3], positions, ray_ids=ray_ids)
            self.self_counts['batches'] += 1
            self.self_counts['removed'] += int(hits.sum())
            data = data[~hits]
        xyz = data[:, 0:3]
        intensity = data[:, 3]
        if has_ring:
            ring = data[:, 4].astype(np.int32)
        else:
            horizontal = np.hypot(xyz[:, 0], xyz[:, 1])
            elevation = np.arctan2(xyz[:, 2], np.maximum(horizontal, 1e-9))
            span = max(self.v_max - self.v_min, 1e-9)
            ring = np.rint((elevation - self.v_min) / span * (self.scan_lines - 1))
            ring = np.clip(ring, 0, self.scan_lines - 1).astype(np.int32)

        # The frame is a single instantaneous snapshot, so every point shares
        # the header stamp (see the module docstring).
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        offset = 0
        if self.mode == "micro_scan":
            # Each sector comes from a DIFFERENT real Gazebo acquisition. No
            # coordinate compensation, interpolation or synthetic scan times.
            if self.last_ns is not None and (stamp_ns <= self.last_ns or
                    stamp_ns - self.last_ns > self.batch_period_ns * 1.5):
                self.pending = []
                self.batch_index = 0
            self.last_ns = stamp_ns
            if self.batch_index == 0:
                self.first_header = copy.deepcopy(msg.header)
                self.first_ns = stamp_ns
            azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
            sector = np.floor((azimuth + np.pi) / (2*np.pi) * self.batch_count)
            sector = np.clip(sector, 0, self.batch_count - 1).astype(int)
            keep = sector == self.batch_index
            xyz, intensity, ring = xyz[keep], intensity[keep], ring[keep]
            offset = stamp_ns - self.first_ns
            if self.micro_publish_mode == "wedge40":
                # E2: each acquisition's sector is one instantaneous scan at
                # its own true stamp. All its points genuinely share this stamp,
                # so no per-point offsets are claimed and no cross-acquisition
                # composition is created. batch_index still advances so that a
                # later fallback to composite starts from the next sector.
                out = CustomMsg()
                out.header = copy.deepcopy(msg.header)
                out.timebase = stamp_ns
                out.point_num = len(xyz)
                out.lidar_id = 0
                out.points = [
                    CustomPoint(
                        offset_time=0,
                        x=float(xyz[i, 0]),
                        y=float(xyz[i, 1]),
                        z=float(xyz[i, 2]),
                        reflectivity=int(max(0, min(255, intensity[i]))),
                        tag=0x10,
                        line=int(ring[i]),
                    )
                    for i in range(len(xyz))
                ]
                self.pub.publish(out)
                self.batch_index = (self.batch_index + 1) % self.batch_count
                return

        out = CustomMsg()
        out.header = copy.deepcopy(msg.header)
        out.timebase = stamp_ns
        out.point_num = len(xyz)
        out.lidar_id = 0
        out.points = [
            CustomPoint(
                offset_time=int(offset),
                x=float(xyz[i, 0]),
                y=float(xyz[i, 1]),
                z=float(xyz[i, 2]),
                reflectivity=int(max(0, min(255, intensity[i]))),
                tag=0x10,
                line=int(ring[i]),
            )
            for i in range(len(xyz))
        ]
        if self.mode == "micro_scan":
            self.pending.extend(out.points)
            self.batch_index += 1
            if self.batch_index < self.batch_count:
                return
            out.header = self.first_header
            out.timebase = self.first_ns
            out.points = self.pending
            out.point_num = len(self.pending)
            self.pending = []
            self.batch_index = 0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarToLivox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
