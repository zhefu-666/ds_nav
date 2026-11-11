#!/usr/bin/env python3
"""Recover base state at LIO measurement time; gate prior or local mapping."""

import copy
import math
import time
from sensor_msgs.msg import JointState
from sensor_state_math import JointHistory, mount_state, recover, TrackingGuard

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def _rotate(q, v):
    """Rotate vector v by the quaternion q (x, y, z, w) without extra deps."""
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    )


class SimStateAdapter(Node):
    def __init__(self):
        super().__init__("sim_state_adapter")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter(
            "gicp_state_topic", "/gicp_relocalization_node/localization_state"
        )
        self.declare_parameter("parent_frame", "camera_init")
        # base_footprint, not body: the URDF owns body beneath its configured
        # sensor parent, so body must keep exactly one parent.
        self.declare_parameter("child_frame", "base_footprint")
        # body -> base_footprint offset, expressed in the body frame:
        # carrier.urdf puts base_footprint 0.10 m below chassis, and body at the
        # chassis origin.
        self.declare_parameter("body_to_child_xyz", [0.0, 0.0, -0.10])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("stale_after", 0.5)
        self.declare_parameter("gicp_state_stale_after", 1.0)
        self.declare_parameter("require_map", True)
        self.declare_parameter("localization_mode", "prior")
        self.declare_parameter("mapping_initialization_seconds", 3.0)
        self.declare_parameter("mapping_initialization_samples", 100)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_history_max_gap", 0.1)
        self.declare_parameter("tracking_guard_enabled", False)
        self.declare_parameter("max_base_speed", .8)
        self.declare_parameter("max_base_yaw_rate", 1.5)
        self.declare_parameter("base_position_jump_slack", .03)
        self.declare_parameter("base_yaw_jump_slack", .03)
        self.guard = None
        if self.get_parameter("tracking_guard_enabled").value:
            self.guard = TrackingGuard(
                stale_after=float(self.get_parameter("stale_after").value),
                max_speed=float(self.get_parameter("max_base_speed").value),
                max_yaw_rate=float(self.get_parameter("max_base_yaw_rate").value),
                position_slack=float(self.get_parameter("base_position_jump_slack").value),
                yaw_slack=float(self.get_parameter("base_yaw_jump_slack").value))
        # Lidar and IMU must be on the same rigid sensor assembly.
        self.declare_parameter("imu_parent_link", "chassis")
        self.mount_link = self.get_parameter("imu_parent_link").value
        self.declare_parameter("lidar_parent_link", self.mount_link)
        if self.mount_link not in ("chassis", "yaw1") or self.get_parameter("lidar_parent_link").value != self.mount_link:
            raise ValueError("Lidar and IMU must share chassis or yaw1 rigid assembly")
        self.mode = self.get_parameter("localization_mode").value
        if self.mode not in ("prior", "mapping"):
            raise ValueError("localization_mode must be prior or mapping")
        self.history = JointHistory(float(self.get_parameter("joint_history_max_gap").value))
        self.initialization_start = None
        self.initialization_count = 0
        self.previous_measurement = None
        self.initialization_last_sample = None
        self.mapping_initialized = False
        self.last_clock_wall = time.monotonic()
        self.joint_error = None

        self.parent = self.get_parameter("parent_frame").value
        self.child = self.get_parameter("child_frame").value
        self.offset = [float(v) for v in self.get_parameter("body_to_child_xyz").value]
        self.map_frame = self.get_parameter("map_frame").value
        self.stale_after = float(self.get_parameter("stale_after").value)
        self.gicp_state_stale_after = float(
            self.get_parameter("gicp_state_stale_after").value
        )
        self.require_map = bool(self.get_parameter("require_map").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_clock_ns = None
        self.epoch_reset = False
        self.latest = None
        self.latest_stamp = None
        self.gicp_state = None
        self.gicp_state_stamp = None

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, qos
        )
        self.create_subscription(
            String,
            self.get_parameter("gicp_state_topic").value,
            self.on_gicp_state,
            10,
        )
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value, self.on_joint, qos)
        self.lidar_pub = self.create_publisher(Odometry, "/sim/lidar_odom", 10)
        self.base_pub = self.create_publisher(Odometry, "/sim/base_odom", 10)
        self.valid_pub = self.create_publisher(Bool, "/sim/localization_valid", 10)
        self.status_pub = self.create_publisher(String, "/sim/localization_status", 10)
        self.create_timer(1.0 / 50.0, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_joint(self, msg):
        if self.mount_link != "yaw1":
            return
        try:
            index = msg.name.index("yaw1_joint")
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if stamp <= 0 or stamp > now + .01 or now-stamp > self.stale_after:
                raise ValueError("invalid_joint_measurement_time")
            self.history.append(stamp, msg.position[index])
            self.joint_error = None
        except (ValueError, IndexError) as exc:
            self.joint_error = str(exc)

    def on_odom(self, msg: Odometry):
        if msg.header.frame_id != self.parent or msg.child_frame_id != "body":
            if self.guard:
                self.guard.latch("unexpected_odometry_frame")
            self.publish_status(False, "unexpected_odometry_frame")
            self.latest = None
            return
        pose = msg.pose.pose
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z,
                  pose.orientation.w]
        norm = sum(v * v for v in values[3:])
        twist = msg.twist.twist
        values.extend([twist.linear.x, twist.linear.y, twist.linear.z,
                       twist.angular.x, twist.angular.y, twist.angular.z])
        if not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 0.01:
            self.latest = None
            if self.guard:
                self.guard.latch("invalid_odometry_pose")
            self.publish_status(False, "invalid_odometry_pose")
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.previous_measurement is not None and stamp <= self.previous_measurement:
            self.latest = None
            if self.guard:
                self.guard.latch("non_monotonic_odometry")
            self.initialization_start = None
            self.initialization_count = 0
            self.publish_status(False, "non_monotonic_odometry")
            return
        self.previous_measurement = stamp
        self.latest = msg
        self.latest_stamp = self.get_clock().now()

    def on_gicp_state(self, msg: String):
        self.gicp_state = msg.data
        self.gicp_state_stamp = self.get_clock().now()

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_clock_ns is not None and now.nanoseconds < self.last_clock_ns:
            self.latest = None
            self.latest_stamp = None
            self.gicp_state = None
            self.gicp_state_stamp = None
            # Both estimators retain their previous epoch internally. A fresh
            # heartbeat is insufficient: restart the localization launch.
            self.epoch_reset = True
        wall_now = time.monotonic()
        if self.last_clock_ns != now.nanoseconds:
            self.last_clock_wall = wall_now
        paused = wall_now - self.last_clock_wall > self.stale_after
        self.last_clock_ns = now.nanoseconds
        if paused:
            self.publish_status(False, "simulation_clock_paused")
            return
        if self.epoch_reset:
            self.publish_status(False, "clock_reset_restart_localization_required")
            return
        if self.guard and self.guard.reason:
            self.publish_status(False, "tracking_lost_restart_localization_required:" + self.guard.reason)
            return
        if self.latest is None:
            if self.guard:
                self.guard.check_time(now.nanoseconds*1e-9)
            self.publish_status(False, "no_odometry")
            return
        age = (self.get_clock().now() - self.latest_stamp).nanoseconds * 1e-9
        measurement_age = (now.nanoseconds - (
            self.latest.header.stamp.sec * 1_000_000_000
            + self.latest.header.stamp.nanosec)) * 1e-9
        # Process a fresh received sample before judging the previous timer
        # sample stale. observe() still rejects real inter-measurement gaps.
        if measurement_age < 0.0 or measurement_age > self.stale_after:
            if self.guard:
                self.guard.check_time(now.nanoseconds*1e-9)
            self.publish_status(False, "invalid_odometry_measurement_time")
            return
        if age < 0.0 or age > self.stale_after:
            if self.guard:
                self.guard.check_time(now.nanoseconds*1e-9)
            self.publish_status(False, f"stale_odometry:{age:.2f}s")
            return

        t = TransformStamped()
        t.header.stamp = self.latest.header.stamp
        t.header.frame_id = self.parent
        t.child_frame_id = self.child
        pose = self.latest.pose.pose
        q = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        twist = self.latest.twist.twist
        velocity = (twist.linear.x, twist.linear.y, twist.linear.z)
        angular = (twist.angular.x, twist.angular.y, twist.angular.z)
        if self.mount_link == "yaw1":
            try:
                if self.joint_error:
                    raise ValueError(self.joint_error)
                measurement_time = self.latest.header.stamp.sec + self.latest.header.stamp.nanosec * 1e-9
                angle, rate = self.history.at(measurement_time)
                state = mount_state(angle, rate)
            except ValueError as exc:
                self.publish_status(False, str(exc))
                return
        else:
            state = (tuple(-v for v in self.offset), (0,0,0,1), (0,0,0), (0,0,0))
        position, orientation, velocity, angular = recover(
            (pose.position.x, pose.position.y, pose.position.z), q, velocity, angular, state)
        if self.guard:
            stamp = self.latest.header.stamp.sec + self.latest.header.stamp.nanosec*1e-9
            reason = self.guard.observe(stamp, position, orientation, velocity, angular)
            if reason:
                prefix = "tracking_lost_restart_localization_required:" if self.guard.reason else "initialization_state_rejected:"
                self.publish_status(False, prefix + reason)
                return
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = orientation
        self.tf_broadcaster.sendTransform(t)
        base = copy.deepcopy(self.latest)
        base.child_frame_id = self.child
        base.pose.pose.position.x, base.pose.pose.position.y, base.pose.pose.position.z = position
        base.pose.pose.orientation = t.transform.rotation
        base.twist.twist.linear.x, base.twist.twist.linear.y, base.twist.twist.linear.z = velocity
        base.twist.twist.angular.x, base.twist.twist.angular.y, base.twist.twist.angular.z = angular
        # ROGMap ray casting starts at odom.pose: publish the sensor origin,
        # never the recovered footprint origin, on its dedicated input.
        lidar = copy.deepcopy(self.latest)
        lidar.child_frame_id = "navigation_laser"
        if self.mount_link == "chassis":
            delta = _rotate(q, (.15, 0, .065))
            lidar.pose.pose.position.x += delta[0]
            lidar.pose.pose.position.y += delta[1]
            lidar.pose.pose.position.z += delta[2]
        self.lidar_pub.publish(lidar)
        if self.mode == "mapping" and not self.mapping_initialized:
            stamp = self.previous_measurement
            if stamp != self.initialization_last_sample:
                self.initialization_last_sample = stamp
                speed = math.sqrt(sum(v*v for v in velocity))
                rotation = math.sqrt(sum(v*v for v in angular))
                if speed < .05 and rotation < .05:
                    if self.initialization_start is None:
                        self.initialization_start = stamp
                    self.initialization_count += 1
                    self.mapping_initialized = (
                        stamp-self.initialization_start >= float(self.get_parameter("mapping_initialization_seconds").value)
                        and self.initialization_count >= int(self.get_parameter("mapping_initialization_samples").value))
                else:
                    self.initialization_start = None
                    self.initialization_count = 0
        # Current LIO does not populate covariance; zero means unspecified here,
        # not a propagated uncertainty or a perfect estimate (see contract).
        base.pose.covariance = [0.0] * 36
        base.twist.covariance = [0.0] * 36
        self.base_pub.publish(base)

        if self.mode == "mapping":
            self.publish_status(self.mapping_initialized, "local_mapping_ready" if self.mapping_initialized else "waiting_stationary_lio_initialization")
            return
        if self.gicp_state is None:
            self.publish_status(False, "waiting_gicp_state")
            return
        gicp_age = (
            self.get_clock().now() - self.gicp_state_stamp
        ).nanoseconds * 1e-9
        if gicp_age < 0.0 or gicp_age > self.gicp_state_stale_after:
            self.publish_status(False, f"stale_gicp_state:{gicp_age:.2f}s")
            return
        if self.gicp_state != "localized":
            self.publish_status(False, f"gicp_{self.gicp_state}")
            return

        map_ready = not self.require_map
        if self.require_map:
            try:
                self.tf_buffer.lookup_transform(self.map_frame, self.parent, rclpy.time.Time())
                map_ready = True
            except Exception:  # noqa: BLE001 - TF lookup exception types vary
                map_ready = False
        if not map_ready:
            self.publish_status(False, "waiting_map_to_camera_init")
            return
        self.publish_status(True, "lio_fresh")

    def publish_status(self, valid: bool, reason: str):
        if self.guard:
            if self.guard.reason:
                valid=False
                reason="tracking_lost_restart_localization_required:" + self.guard.reason
            elif valid:
                self.guard.mark_valid()
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(String(data=reason))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SimStateAdapter()
    stopping = False

    def request_stop(_signum, _frame):
        # Do not raise through pybind take_message: Humble can translate the
        # interrupted conversion into RuntimeError instead of KeyboardInterrupt.
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
