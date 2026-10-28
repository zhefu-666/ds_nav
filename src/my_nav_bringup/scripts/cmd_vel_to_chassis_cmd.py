#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    from rmoss_interfaces.msg import ChassisCmd
except ImportError:
    ChassisCmd = None


class CmdVelToChassisCmd(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_chassis_cmd")

        self.robot_name = self.declare_parameter("robot_name", "red_standard_robot1").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.chassis_cmd_topic = self.declare_parameter(
            "chassis_cmd_topic",
            f"/{self.robot_name}/robot_base/chassis_cmd",
        ).value

        self._warned_missing_rmu = False

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.on_cmd_vel,
            10,
        )

        self.chassis_cmd_pub = None
        if ChassisCmd is not None:
            self.chassis_cmd_pub = self.create_publisher(ChassisCmd, self.chassis_cmd_topic, 10)
            self.get_logger().info(
                f"Bridging {self.cmd_vel_topic} -> {self.chassis_cmd_topic} for {self.robot_name}"
            )
        else:
            self.get_logger().warning("rmoss_interfaces is not available; cmd_vel will not be forwarded.")

    def on_cmd_vel(self, twist: Twist) -> None:
        if ChassisCmd is None or self.chassis_cmd_pub is None:
            if not self._warned_missing_rmu:
                self.get_logger().warning(
                    "rmoss_interfaces.msg.ChassisCmd is missing, so the RMU bridge is idle."
                )
                self._warned_missing_rmu = True
            return

        msg = ChassisCmd()
        if hasattr(msg, "FOLLOW_GIMBAL"):
            msg.type = msg.FOLLOW_GIMBAL
        elif hasattr(ChassisCmd, "FOLLOW_GIMBAL"):
            msg.type = ChassisCmd.FOLLOW_GIMBAL

        msg.twist.linear.x = twist.linear.x
        msg.twist.linear.y = twist.linear.y
        msg.twist.angular.z = twist.angular.z
        self.chassis_cmd_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CmdVelToChassisCmd()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
