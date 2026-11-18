#!/usr/bin/env python3
"""Bound the simulation startup wait without replacing Nav2 lifecycle/bonds."""
import time

import rclpy
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.create_node('sim_navigation_startup_watchdog')
    client = node.create_client(Trigger, '/lifecycle_manager_sim_navigation/is_active')
    deadline = time.monotonic() + 60.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=.5):
                continue
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
            if future.done() and future.result() is not None and future.result().success:
                node.get_logger().info('Navigation lifecycle startup completed')
                return 0
            client.remove_pending_request(future)
            rclpy.spin_once(node, timeout_sec=.25)
        node.get_logger().error('Navigation startup exceeded 60 s; stopping simulation safely')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
