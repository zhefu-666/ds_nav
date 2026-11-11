#!/usr/bin/env python3
"""Exit successfully only after fresh localization readiness is observed."""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node('sim_wait_ready')
    node.declare_parameter('timeout', 180.)
    ready = [False, 0.]
    def receive(msg):
        ready[:] = [msg.data, time.monotonic()]
    node.create_subscription(Bool, '/sim/localization_valid', receive, 10)
    deadline = time.monotonic()+float(node.get_parameter('timeout').value)
    code = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if ready[0] and time.monotonic()-ready[1] < .3:
                node.get_logger().info('Localization ready; navigation may be created.')
                code = 0
                break
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
