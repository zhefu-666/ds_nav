import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    bringup_dir = get_package_share_directory("my_nav_bringup")
    simulator_dir = get_package_share_directory("rmu_gazebo_simulator")

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_demo_map_server = LaunchConfiguration("use_demo_map_server")
    use_static_map_odom = LaunchConfiguration("use_static_map_odom")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_yaw = LaunchConfiguration("goal_yaw")

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="red_standard_robot1",
        description="RMU robot namespace to drive.",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )
    declare_use_demo_map_server_cmd = DeclareLaunchArgument(
        "use_demo_map_server",
        default_value="false",
        description="Use the built-in demo occupancy grid instead of the RMUL 2026 static map.",
    )
    declare_use_static_map_odom_cmd = DeclareLaunchArgument(
        "use_static_map_odom",
        default_value="true",
        description="Publish a static map->odom transform until localization exists.",
    )
    declare_goal_x_cmd = DeclareLaunchArgument(
        "goal_x",
        default_value="0.0",
        description="Demo goal X position (mid-field on rmul_2026).",
    )
    declare_goal_y_cmd = DeclareLaunchArgument(
        "goal_y",
        default_value="-2.0",
        description="Demo goal Y position (mid-field on rmul_2026).",
    )
    declare_goal_yaw_cmd = DeclareLaunchArgument(
        "goal_yaw",
        default_value="0.0",
        description="Demo goal yaw.",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 alongside the sim.",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "lidar_preview.rviz"),
        description="Path to the RViz config file.",
    )

    rmu_simulator_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_dir, "launch", "bringup_sim.launch.py")
        )
    )

    navigation_cmd = GroupAction([
        PushRosNamespace(robot_name),
        SetRemap(src="/tf", dst="tf"),
        SetRemap(src="/tf_static", dst="tf_static"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "navigation_core.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_demo_map_server": use_demo_map_server,
                "use_fake_hardware": "false",
                "use_static_map_odom": use_static_map_odom,
                "map_topic": "/map",
                "odom_topic": ["/", robot_name, "/chassis_odometry_gt"],
                "path_topic": "/planned_path",
                "cmd_vel_topic": "/cmd_vel",
                "map_frame": "map",
                "base_frame": "chassis",
                "odom_frame": "odom",
                "goal_x": goal_x,
                "goal_y": goal_y,
                "goal_yaw": goal_yaw,
            }.items(),
        ),
        Node(
            package="my_nav_bringup",
            executable="cmd_vel_to_chassis_cmd.py",
            name="cmd_vel_to_chassis_cmd",
            output="screen",
            parameters=[{
                "robot_name": robot_name,
                "cmd_vel_topic": "/cmd_vel",
            }],
        ),
    ])

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/scan", ["/", robot_name, "/rplidar_a2/scan"]),
        ],
    )

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        declare_use_demo_map_server_cmd,
        declare_use_static_map_odom_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_cmd,
        declare_goal_x_cmd,
        declare_goal_y_cmd,
        declare_goal_yaw_cmd,
        rmu_simulator_cmd,
        navigation_cmd,
        rviz_cmd,
    ])
