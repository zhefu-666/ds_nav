import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("my_nav_bringup")
    default_map_yaml = os.path.join(
        bringup_share, "maps", "simulation", "rmul_2026.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_demo_map_server = LaunchConfiguration("use_demo_map_server")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_static_map_odom = LaunchConfiguration("use_static_map_odom")
    map_yaml = LaunchConfiguration("map_yaml")

    map_topic = LaunchConfiguration("map_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    path_topic = LaunchConfiguration("path_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")

    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_yaw = LaunchConfiguration("goal_yaw")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    declare_use_demo_map_server_cmd = DeclareLaunchArgument(
        "use_demo_map_server",
        default_value="false",
        description="Publish the built-in demo occupancy grid instead of the static RMUL map.",
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map_yaml",
        default_value=default_map_yaml,
        description="Path to the static map YAML consumed by static_map_server.",
    )
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Use the local fake base driver instead of a real vehicle backend.",
    )
    declare_use_static_map_odom_cmd = DeclareLaunchArgument(
        "use_static_map_odom",
        default_value="true",
        description="Publish a static map->odom transform for development.",
    )

    declare_map_topic_cmd = DeclareLaunchArgument(
        "map_topic",
        default_value="/map",
        description="Occupancy grid topic.",
    )
    declare_odom_topic_cmd = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="Odometry topic used by the planner.",
    )
    declare_path_topic_cmd = DeclareLaunchArgument(
        "path_topic",
        default_value="/planned_path",
        description="Planned path topic.",
    )
    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Base velocity command topic.",
    )
    declare_map_frame_cmd = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Global frame for planning and control.",
    )
    declare_base_frame_cmd = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame.",
    )
    declare_odom_frame_cmd = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="Odometry frame.",
    )

    declare_goal_x_cmd = DeclareLaunchArgument(
        "goal_x",
        default_value="2.8",
        description="Demo goal X position.",
    )
    declare_goal_y_cmd = DeclareLaunchArgument(
        "goal_y",
        default_value="2.8",
        description="Demo goal Y position.",
    )
    declare_goal_yaw_cmd = DeclareLaunchArgument(
        "goal_yaw",
        default_value="0.0",
        description="Demo goal yaw.",
    )

    demo_map_server_cmd = Node(
        condition=IfCondition(use_demo_map_server),
        package="my_global_planner",
        executable="demo_map_server",
        name="demo_map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": map_frame,
            "resolution": 0.05,
            "width": 160,
            "height": 160,
            "origin_x": -4.0,
            "origin_y": -4.0,
            "wall_thickness_cells": 4,
        }],
    )

    static_map_server_cmd = Node(
        condition=UnlessCondition(use_demo_map_server),
        package="my_global_planner",
        executable="static_map_server",
        name="static_map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_path": map_yaml,
            "map_topic": map_topic,
            "frame_id": map_frame,
            "republish_period": 0.0,
        }],
    )

    planner_cmd = Node(
        package="my_global_planner",
        executable="planner_node",
        name="my_global_planner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_topic": map_topic,
            "odom_topic": odom_topic,
            "goal_topic": "/goal_pose",
            "path_topic": path_topic,
            "map_frame": map_frame,
            "planning_frequency": 2.0,
            "obstacle_threshold": 50,
            "allow_unknown": False,
            "use_diagonal": True,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_yaw": goal_yaw,
        }],
    )

    controller_cmd = Node(
        package="my_nav_control",
        executable="path_follower_node",
        name="path_follower_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_frame": map_frame,
            "base_frame": base_frame,
            "path_topic": path_topic,
            "cmd_vel_topic": cmd_vel_topic,
            "control_frequency": 20.0,
            "lookahead_distance": 0.6,
            "goal_tolerance": 0.20,
            "linear_speed": 0.45,
            "max_linear_speed": 0.60,
            "max_angular_speed": 1.5,
            "heading_stop_threshold": 1.05,
        }],
    )

    fake_base_driver_cmd = Node(
        condition=IfCondition(use_fake_hardware),
        package="my_nav_control",
        executable="fake_base_driver_node",
        name="fake_base_driver_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "cmd_vel_topic": cmd_vel_topic,
            "odom_topic": odom_topic,
            "publish_rate": 50.0,
            "cmd_timeout": 0.25,
            "max_linear_speed": 0.8,
            "max_angular_speed": 2.0,
            "initial_x": 0.0,
            "initial_y": 0.0,
            "initial_yaw": 0.0,
        }],
    )

    static_map_to_odom_cmd = Node(
        condition=IfCondition(use_static_map_odom),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", map_frame,
            "--child-frame-id", odom_frame,
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        declare_use_sim_time_cmd,
        declare_use_demo_map_server_cmd,
        declare_map_yaml_cmd,
        declare_use_fake_hardware_cmd,
        declare_use_static_map_odom_cmd,
        declare_map_topic_cmd,
        declare_odom_topic_cmd,
        declare_path_topic_cmd,
        declare_cmd_vel_topic_cmd,
        declare_map_frame_cmd,
        declare_base_frame_cmd,
        declare_odom_frame_cmd,
        declare_goal_x_cmd,
        declare_goal_y_cmd,
        declare_goal_yaw_cmd,
        demo_map_server_cmd,
        static_map_server_cmd,
        planner_cmd,
        controller_cmd,
        fake_base_driver_cmd,
        static_map_to_odom_cmd,
    ])
