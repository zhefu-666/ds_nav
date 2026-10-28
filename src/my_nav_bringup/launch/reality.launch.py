import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("my_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_demo_map_server = LaunchConfiguration("use_demo_map_server")
    use_static_map_odom = LaunchConfiguration("use_static_map_odom")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_yaw = LaunchConfiguration("goal_yaw")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use wall clock for the dry-run reality stack.",
    )
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Use the local fake base driver until a real base backend is available.",
    )
    declare_use_demo_map_server_cmd = DeclareLaunchArgument(
        "use_demo_map_server",
        default_value="true",
        description="Use the built-in demo occupancy grid for offline dry runs.",
    )
    declare_use_static_map_odom_cmd = DeclareLaunchArgument(
        "use_static_map_odom",
        default_value="true",
        description="Publish a static map->odom transform until localization is connected.",
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

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation_core_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "navigation_core.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_demo_map_server": use_demo_map_server,
            "use_fake_hardware": use_fake_hardware,
            "use_static_map_odom": use_static_map_odom,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "goal_yaw": goal_yaw,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_fake_hardware_cmd,
        declare_use_demo_map_server_cmd,
        declare_use_static_map_odom_cmd,
        declare_goal_x_cmd,
        declare_goal_y_cmd,
        declare_goal_yaw_cmd,
        robot_description_cmd,
        navigation_core_cmd,
    ])
