"""Spawn the RMUL 2027 robot without any kaohe runtime dependency."""

import os
import json
import math
import runpy
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def _load_module(share: Path, name: str):
    scripts = share / "scripts"
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    return runpy.run_path(str(scripts / name))


def spawn(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    drive_model = LaunchConfiguration("drive_model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)

    sim_share = Path(get_package_share_directory("huananhu_sim_bringup"))
    res_share = Path(get_package_share_directory("huananhu_sim_resources"))

    world_config = yaml.safe_load((sim_share / "config" / "world.yaml").read_text())
    robots = world_config["robots"][world_config["world"]]
    robot = dict(robots[0])
    spawn_pose = LaunchConfiguration("spawn_pose").perform(context)
    if spawn_pose:
        values = json.loads(spawn_pose)
        if not isinstance(values, list) or len(values) != 4 or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError("spawn_pose must be finite JSON [x,y,z,yaw]")
        robot.update(dict(zip(("x_pose", "y_pose", "z_pose", "yaw"), map(str, values))))

    robot_xml = (res_share / "models" / "low_profile_robot" / "model.sdf").read_text()
    urdf_xml = (res_share / "urdf" / "carrier.urdf").read_text()

    if robot_model == "huananhu":
        nav_model = _load_module(res_share, "navigation_model.py")
        robot_xml, _carrier_xml = nav_model["compose"](str(res_share), robot_xml, robot_name)
        urdf_xml = nav_model["compose_urdf"](str(res_share), urdf_xml)
        if drive_model == "rollers":
            rollers = _load_module(res_share, "roller_wheels.py")
            robot_xml = rollers["add_rollers"](robot_xml, str(res_share), 0.8, 0.0)
            urdf_xml = rollers["append_roller_urdf"](urdf_xml, robot_xml)
            gain=float(LaunchConfiguration("wheel_integral_gain").perform(context))
            if not math.isfinite(gain) or not 0.<=gain<=1.:
                raise ValueError("wheel_integral_gain must be finite in [0,1]")
            proportional=float(LaunchConfiguration("wheel_proportional_gain").perform(context))
            if not math.isfinite(proportional) or not 0.<proportional<=2.:
                raise ValueError("wheel_proportional_gain must be finite in (0,2]")
            if gain!=1. or proportional!=.30:
                root=ET.fromstring(robot_xml)
                plugin=root.find(".//plugin[@name='kaohe::WheelTractionDrive']")
                if gain!=1.:
                    ET.SubElement(plugin,"integral_gain").text=str(gain)
                if proportional!=.30:
                    ET.SubElement(plugin,"proportional_gain").text=str(proportional)
                robot_xml=ET.tostring(root,encoding="unicode")

    sampling_mode = LaunchConfiguration("sampling_mode").perform(context)
    if sampling_mode == "micro_scan":
        root = ET.fromstring(robot_xml)
        sensor = root.find(".//sensor[@name='navigation_lidar3d']")
        if sensor is None:
            raise RuntimeError("Micro-scan lidar sensor missing")
        sensor.find("update_rate").text = "40"
        robot_xml = ET.tostring(root, encoding="unicode")

    with tempfile.NamedTemporaryFile(
        mode="w", prefix="huananhu_sim_", suffix=".sdf", delete=False
    ) as sdf_file:
        sdf_file.write(robot_xml)

    bridge_source = str(sim_share / "config" / "bridge.yaml")
    bridge_after_replace = ReplaceString(
        source_file=bridge_source, replacements={"<robot_name>": robot_name, "<sensor_parent>": "yaw1" if robot_model == "huananhu" else "chassis"}
    )

    actions = [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file",
                sdf_file.name,
                "-name",
                robot_name,
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"use_sim_time": True, "robot_description": urdf_xml}],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{"config_file": bridge_after_replace}],
            output="screen",
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="huananhu",
                choices=["huananhu", "low_profile_robot"],
            ),
            DeclareLaunchArgument(
                "drive_model", default_value="proxy", choices=["proxy", "rollers"]
            ),
            DeclareLaunchArgument("robot_name", default_value="huananhu"),
            DeclareLaunchArgument("wheel_proportional_gain", default_value="0.30",
                                  description="Optional simulation wheel proportional gain; normal default unchanged"),
            DeclareLaunchArgument("wheel_integral_gain", default_value="1.0",
                                  description="Optional simulation wheel integral gain; normal default unchanged"),
            DeclareLaunchArgument("spawn_pose", default_value="",
                                  description="Optional simulation spawn JSON [x,y,z,yaw]"),
            DeclareLaunchArgument("sampling_mode", default_value="snapshot",
                                  choices=["snapshot", "micro_scan"]),
            OpaqueFunction(function=spawn),
        ]
    )
