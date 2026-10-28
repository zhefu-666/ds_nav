import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext) -> list:
    robot_description_file = context.launch_configurations["robot_description_file"]
    use_sim_time = context.launch_configurations["use_sim_time"].lower() in ("true", "1", "yes", "on")

    with open(robot_description_file, "r", encoding="utf-8") as stream:
        robot_description_xml = stream.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_xml,
            }],
        )
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("my_nav_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    declare_robot_description_file_cmd = DeclareLaunchArgument(
        "robot_description_file",
        default_value=os.path.join(bringup_dir, "urdf", "demo_robot.urdf"),
        description="Path to the robot URDF used by robot_state_publisher.",
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_robot_description_file_cmd,
        OpaqueFunction(function=launch_setup),
    ])
