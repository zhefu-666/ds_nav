"""Standalone simulation Nav2 baseline; no real-robot component container.

Requires estimator TF and /sim/base_odom before activation. The parent simulation
launch owns map->camera_init and the gated global-to-body command adapter.
"""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = Path(get_package_share_directory('huananhu_sim_bringup'))
    mode = LaunchConfiguration('map_mode').perform(context)
    if mode not in ('prior', 'mapping'):
        raise ValueError('map_mode must be prior or mapping')
    params_path = Path(LaunchConfiguration('nav_params_file').perform(context))
    params = yaml.safe_load(params_path.read_text())
    map_path = Path(LaunchConfiguration('map_yaml').perform(context)).resolve()
    if mode == 'prior' and not map_path.is_file():
        raise FileNotFoundError(map_path)
    # Resolve paths from the installed package, independent of launch cwd.
    bt = params['bt_navigator']['ros__parameters']
    bt['default_nav_to_pose_bt_xml'] = str(share / 'behavior_tree/navigate_to_pose.xml')
    bt['default_nav_through_poses_bt_xml'] = str(share / 'behavior_tree/navigate_through_poses.xml')
    params['map_server']['ros__parameters']['yaml_filename'] = str(map_path)
    planner = params['planner_server']['ros__parameters']['MincoPlanner']
    planner['planner_mode'] = 'PRIORMAP' if mode == 'prior' else 'EXPLORATION'
    planner['rog_map']['projection']['prior_map'].update(
        enable=mode == 'prior', yaml_path=str(map_path) if mode == 'prior' else '')
    if mode == 'mapping':
        for key in ('local_costmap', 'global_costmap'):
            costmap = params[key][key]['ros__parameters']
            costmap['plugins'] = [p for p in costmap['plugins'] if p != 'static_layer']
            costmap.pop('static_layer', None)
            costmap['rolling_window'] = True
        # No map server / prior raster in online mode; observed ROGMap free space
        # is authoritative for MINCO, with unknown space rejected by exploration.
    # A shared ROS parameter file is required so each costmap child node gets its
    # own section. Register cleanup rather than retaining generated configuration.
    with tempfile.NamedTemporaryFile(mode='w', prefix='huananhu_nav_', suffix='.yaml',
                                     delete=False) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        runtime_path = Path(stream.name)

    def cleanup(_context):
        runtime_path.unlink(missing_ok=True)
        return []

    common = {'output': 'screen', 'parameters': [str(runtime_path)]}
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
    actions.extend([
        Node(package='huananhu_sim_bringup', executable='simulation_planner_server',
             name='planner_server', **common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             remappings=[('/aft_mapped_to_init', '/sim/base_odom'),
                         ('cmd_vel', '/sim/nav_cmd_vel_global'),
                         ('/cmd_vel_mpc', '/sim/debug/cmd_vel_mpc_global')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
    ])
    lifecycle_nodes = ['planner_server', 'controller_server', 'bt_navigator', 'waypoint_follower']
    if mode == 'prior':
        actions.append(Node(package='nav2_map_server', executable='map_server', name='map_server', **common))
        lifecycle_nodes.insert(0, 'map_server')
    autostart = LaunchConfiguration('nav_autostart').perform(context).lower()
    if autostart not in ('true', 'false'):
        raise ValueError('nav_autostart must be true or false')
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_manager_sim_navigation', output='screen',
                        parameters=[{'use_sim_time': True, 'autostart': autostart == 'true',
                                     'node_names': lifecycle_nodes}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('huananhu_sim_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('map_mode', default_value='prior',
                              description='prior: existing map; mapping: online ROGMap without static prior'),
        DeclareLaunchArgument('nav_params_file', default_value=str(share / 'config/navigation_sim.yaml')),
        DeclareLaunchArgument('map_yaml', default_value=str(share / 'maps/rmul_2027.yaml')),
        DeclareLaunchArgument('nav_autostart', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
