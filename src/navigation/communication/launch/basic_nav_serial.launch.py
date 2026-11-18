from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('rmul_v1', default_value='false'),
        Node(
            package='communication',
            executable='basic_nav_serial',
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'rmul_v1': ParameterValue(LaunchConfiguration('rmul_v1'), value_type=bool),
            }],
        ),
    ])
