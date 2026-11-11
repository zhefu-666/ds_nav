#!/usr/bin/env python3
"""Convert this project's camera_init-frame MPC command to chassis twist.

Only the standard Nav2 controller output is consumed. The MPC diagnostic
publisher is deliberately not an actuator source. No truth inputs are used.
"""
import math
import time

import signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def body_command(command, orientation):
    values = (*command, *orientation)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite command or orientation")
    x, y, z, w = orientation
    if abs(x*x + y*y + z*z + w*w - 1.0) > 0.01:
        raise ValueError("invalid orientation")
    yaw = math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
    vx, vy, wz = command
    return (math.cos(yaw)*vx + math.sin(yaw)*vy,
            -math.sin(yaw)*vx + math.cos(yaw)*vy, wz)


class NavCommandAdapter(Node):
    def __init__(self):
        super().__init__('sim_nav_command_adapter')
        self.command = None
        self.odom = None
        self.command_at = self.odom_at = -math.inf
        self.pub = self.create_publisher(Twist, '/sim/auto_cmd_vel', 10)
        self.create_subscription(Twist, '/sim/nav_cmd_vel_global', self.on_command, 10)
        self.create_subscription(Odometry, '/sim/base_odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        self.command = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.command_at = time.monotonic()

    def on_odom(self, msg):
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'base_footprint':
            self.odom = None
            return
        self.odom = msg
        self.odom_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        output = Twist()
        if self.command is not None and self.odom is not None and \
                now-self.command_at <= .25 and now-self.odom_at <= .5:
            stamp = self.odom.header.stamp
            age = (self.get_clock().now().nanoseconds -
                   stamp.sec*1_000_000_000-stamp.nanosec)*1e-9
            if 0 <= age <= .5:
                q = self.odom.pose.pose.orientation
                try:
                    output.linear.x, output.linear.y, output.angular.z = body_command(
                        self.command, (q.x, q.y, q.z, q.w))
                except ValueError:
                    self.command = None
        self.pub.publish(output)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavCommandAdapter()
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
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
