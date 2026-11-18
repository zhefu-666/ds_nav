"""Single entry for the RMUL 2027 + 华南虎 simulation.

Starts Gazebo Fortress, the articulated robot, TF/bridge plumbing and
optionally the project's Point-LIO + GICP localization chain. Nothing here
reads from the kaohe workspace: every path comes from this package's share
directory or from installed ROS packages.
"""

import os
import runpy
import shutil
import hashlib
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("huananhu_sim_bringup")
    res_share = get_package_share_directory("huananhu_sim_resources")
    defaults = json.loads((Path(sim_share) / "config/simulation_defaults.json").read_text())

    # Render only Gazebo/RViz on NVIDIA; algorithm processes retain their CPU environment.
    render_env = {
        "__NV_PRIME_RENDER_OFFLOAD": "1",
        "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        "__VK_LAYER_NV_optimus": "NVIDIA_only",
    }

    scene = LaunchConfiguration("scene")
    world_sdf = PathJoinSubstitution([res_share, "worlds", [scene, "_world.sdf"]])
    gui_config = os.path.join(res_share, "ign", "gui.config")

    headless = LaunchConfiguration("headless")
    localization = LaunchConfiguration("localization")
    physics_test = LaunchConfiguration("physics_test")
    map_mode = LaunchConfiguration("map_mode")
    navigation = LaunchConfiguration("navigation")
    rviz = LaunchConfiguration("rviz")
    robot_model = LaunchConfiguration("robot_model")
    drive_model = LaunchConfiguration("drive_model")
    robot_name = LaunchConfiguration("robot_name")

    # The prior map and GICP initial pose describe rmul_2027 only. The
    # drawing-dimension fixtures use the diagnostic physics-test gate instead.
    localization_ok = PythonExpression([
        "'", localization, "' == 'true' and '", physics_test, "' == 'false'",
        " and '", scene, "' == 'rmul_2027'",
    ])
    posture_ok = PythonExpression(["'", robot_model, "' == 'huananhu'"])
    prior_ok = PythonExpression(["(", localization_ok, ") and '", map_mode, "' == 'prior'"])
    mapping_ok = PythonExpression(["(", localization_ok, ") and '", map_mode, "' == 'mapping'"])
    nav_ok = PythonExpression(["(", localization_ok, ") and '", navigation, "' == 'true'"])
    sensor_parent = PythonExpression(["'yaw1' if '", robot_model, "' == 'huananhu' else 'chassis'"])
    sim_args = PythonExpression(["'-r -s' if '", headless, "' == 'true' else '-r'"])

    def make_gazebo(context):
        if navigation.perform(context).lower() == 'true' and (
                localization.perform(context).lower() != 'true'
                or physics_test.perform(context).lower() == 'true'
                or scene.perform(context) != 'rmul_2027'):
            raise ValueError('Navigation requires rmul_2027, localization and the formal gate; set navigation:=false for diagnostics')
        engine = LaunchConfiguration('render_engine').perform(context)
        engine_library = LaunchConfiguration('render_engine_library').perform(context)
        if engine_library == 'auto':
            engine_library = ''
            if engine == 'ogre':
                runtime = Path(sim_share) / 'runtime/rendering'
                hashes = json.loads((runtime / 'SHA256.json').read_text())
                library = runtime / 'libignition-rendering6-ogre.so.6.6.4'
                if hashlib.sha256(library.read_bytes()).hexdigest() != hashes[library.name]:
                    raise ValueError('Packaged Ogre renderer checksum mismatch')
                engine_library = str(library.resolve())
        if engine_library:
            library = Path(engine_library)
            if engine != 'ogre' or not library.is_absolute() or not library.is_file():
                raise ValueError('render_engine_library requires ogre and an existing absolute library path')
            engine = str(library.resolve())
        factor = float(LaunchConfiguration('real_time_factor').perform(context))
        if not 0 < factor <= 2:
            raise ValueError('real_time_factor must be in (0,2]')
        sdf_path = world_sdf.perform(context)
        cleanup = []
        if engine != 'ogre2' or factor != 1.0:
            root = ET.parse(sdf_path)
            sensors = root.find(".//plugin[@name='ignition::gazebo::systems::Sensors']/render_engine")
            if sensors is None:
                raise ValueError('World is missing its sensor renderer')
            sensors.text = engine
            world = root.find('world')
            physics = world.find('physics')
            if physics is None:
                physics = ET.SubElement(world, 'physics', name='simulation_pacing', type='ignored')
                ET.SubElement(physics, 'max_step_size').text = '0.001'
            pacing = physics.find('real_time_factor')
            if pacing is None:
                pacing = ET.SubElement(physics, 'real_time_factor')
            pacing.text = str(factor)
            with tempfile.NamedTemporaryFile(prefix='huananhu_renderer_', suffix='.sdf', delete=False) as stream:
                runtime_world = Path(stream.name)
            root.write(runtime_world, encoding='utf-8', xml_declaration=True)
            sdf_path = str(runtime_world)
            cleanup = [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
                function=lambda _context: (runtime_world.unlink(missing_ok=True), [])[1])]))]
        # Preserve ros_gz_sim's exported resource/plugin paths, but remove its
        # shell=True wrapper so launch signals reach the actual Gazebo process.
        upstream = runpy.run_path(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))
        model_paths, plugin_paths = upstream['GazeboRosPaths'].get_paths()
        gazebo_env = dict(render_env)
        for name in ('GZ_SIM_SYSTEM_PLUGIN_PATH', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH'):
            gazebo_env[name] = os.pathsep.join([
                os.environ.get(name, ''), os.environ.get('LD_LIBRARY_PATH', ''), plugin_paths])
        for name in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
            gazebo_env[name] = os.pathsep.join([os.environ.get(name, ''), model_paths])
        return cleanup + [ExecuteProcess(
            cmd=[shutil.which('ign'), 'gazebo', *sim_args.perform(context).split(),
                 sdf_path, '--gui-config', gui_config, '--force-version', '6'],
            additional_env=gazebo_env, output='screen')]
    gazebo = OpaqueFunction(function=make_gazebo)

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "robot_model": robot_model,
            "drive_model": drive_model,
            "robot_name": robot_name,
            "sampling_mode": LaunchConfiguration("sampling_mode"),
            "spawn_pose": LaunchConfiguration("spawn_pose"),
            "wheel_integral_gain": LaunchConfiguration("wheel_integral_gain"),
            "wheel_proportional_gain": LaunchConfiguration("wheel_proportional_gain"),
        }.items(),
    )

    lidar_adapter = Node(
        package="huananhu_sim_bridge",
        executable="sim_lidar_to_livox.py",
        # Tiny 3-column hull products otherwise spawn a large BLAS thread pool.
        additional_env={'OPENBLAS_NUM_THREADS': '1'},
        parameters=[
            {
                "sampling_mode": LaunchConfiguration("sampling_mode"),
                "input_topic": "/lidar/points",
                "output_topic": "/livox/lidar",
                "scan_lines": 32,
                "vertical_min_angle": -0.12217304764,
                "vertical_max_angle": 0.90757121104,
                "micro_publish_mode": LaunchConfiguration("micro_publish_mode"),
                "micro_batches": LaunchConfiguration("micro_batches"),
                "self_filter": ParameterValue(PythonExpression([
                    "'", robot_model, "' == 'huananhu' and '",
                    LaunchConfiguration('self_filter'), "' == 'true'"]), value_type=bool),
                "self_filter_resource_dir": res_share,
            }
        ],
        condition=IfCondition(localization_ok),
        output="screen",
    )

    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        parameters=[
            PathJoinSubstitution([sim_share, 'config', PythonExpression([
                "'pointlio_sim.yaml' if '", robot_model,
                "' == 'huananhu' else 'pointlio_carrier.yaml'"])]),
            {"use_sim_time": True},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        condition=IfCondition(localization_ok),
        output="screen",
    )

    state_adapter = Node(
        package="huananhu_sim_bridge",
        executable="sim_state_adapter.py",
        parameters=[{"use_sim_time": True, "imu_parent_link": sensor_parent,
                     "lidar_parent_link": sensor_parent,
                     "localization_mode": map_mode, "tracking_guard_enabled": True}],
        condition=IfCondition(localization_ok),
        output="screen",
    )

    def make_gicp(context):
        initial = ([-4.94424028099487, -0.163594690492484, 0.278643863403111, 0., 0., 0.]
                   if robot_model.perform(context) == 'huananhu' else [-5., 0., .18, 0., 0., 0.])
        bundle = load_bundle(context)
        if bundle:
            initial = bundle['relocalization_initial_pose']
        override = LaunchConfiguration('initial_imu_pose').perform(context)
        if (prior_ok.perform(context).lower() == 'true'
                and LaunchConfiguration('spawn_pose').perform(context) and not override and not bundle):
            raise ValueError('Custom prior-mode spawn requires explicit initial_imu_pose')
        if override:
            initial = json.loads(override)
        if len(initial) != 6 or not all(math.isfinite(float(v)) for v in initial):
            raise ValueError('initial_imu_pose requires six finite values')
        return [Node(
        package="icp_relocalization",
        executable="gicp_node",
        name="gicp_relocalization_node",
        parameters=[
            os.path.join(sim_share, "config", "gicp_sim.yaml"),
            {
                "use_sim_time": True,
                # 3D prior sampled from the same visual STL seen by GPU lidar by
                # scripts/generate_prior_map.py; see maps/prior_map_manifest.json.
                "target_pcd_file": bundle['pcd_path'] if bundle else os.path.join(sim_share, "maps", "prior_map.pcd"),
                # yaw1 measurement origin from the explicit sensor mount;
                # this is a configured spawn prior, never a truth subscription.
                "initial_pose": initial,
            },
        ],
        condition=IfCondition(prior_ok),
        output="screen",
        )]
    gicp = OpaqueFunction(function=make_gicp)

    posture_controller = Node(
        package="huananhu_sim_bridge",
        executable="sim_posture_controller.py",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(posture_ok),
        output="screen",
    )

    # Exactly one actuator owner: the formal localization gate or the
    # DIAGNOSTIC physics-test gate, never both.
    command_gate = Node(
        package="huananhu_sim_bridge", executable="sim_command_gate.py",
        parameters=[{"use_sim_time": True}], output="screen",
        condition=UnlessCondition(physics_test),
    )
    physics_gate = Node(
        package="huananhu_sim_bridge", executable="sim_physics_test_gate.py",
        parameters=[{"use_sim_time": True}], output="screen",
        condition=IfCondition(physics_test),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        additional_env=render_env,
        arguments=["-d", PathJoinSubstitution([sim_share, "rviz", LaunchConfiguration("rviz_config")])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        output="screen",
    )
    local_map_origin = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='local_mapping_origin',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': True}], condition=IfCondition(mapping_ok),
    )
    def load_bundle(context):
        source = LaunchConfiguration('saved_map_manifest').perform(context)
        if not source:
            return None
        if map_mode.perform(context) != 'prior':
            raise ValueError('saved_map_manifest is a prior-mode input')
        path = Path(source).resolve()
        bundle = json.loads(path.read_text())
        if (bundle.get('status') != 'VALID_MAP_BUNDLE' or bundle.get('frame') != 'camera_init'
                or bundle.get('schema_version') != 1 or bundle.get('map_gauge') != 'local_initial_lio'):
            raise ValueError('Invalid or unsupported saved map bundle')
        for kind in ('pcd', 'yaml', 'image'):
            artifact = bundle['files'][kind]
            artifact_path = path.parent / artifact['path']
            if hashlib.sha256(artifact_path.read_bytes()).hexdigest() != artifact['sha256']:
                raise ValueError('Saved map checksum mismatch: ' + kind)
            bundle[kind + '_path'] = str(artifact_path.resolve())
        map_yaml_path = Path(bundle['yaml_path'])
        map_yaml = yaml.safe_load(map_yaml_path.read_text())
        if (map_yaml_path.parent / map_yaml['image']).resolve() != Path(bundle['image_path']):
            raise ValueError('Saved YAML image does not match the checksummed image')
        return bundle

    def start_navigation(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='ERROR: localization did not become ready; navigation was not created.')]
        bundle = load_bundle(context)
        arguments = {'map_mode': map_mode}
        if bundle:
            arguments['map_yaml'] = bundle['yaml_path']
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'navigation.launch.py')),
            launch_arguments=arguments.items())]

    navigation_wait = Node(
        package='huananhu_sim_bridge', executable='sim_wait_ready.py',
        parameters=[{'use_sim_time': True, 'timeout': 180.}],
        condition=IfCondition(nav_ok), output='screen',
    )
    navigation_ready_handler = RegisterEventHandler(OnProcessExit(
        target_action=navigation_wait, on_exit=start_navigation))
    nav_command_adapter = Node(
        package='huananhu_sim_bridge', executable='sim_nav_command_adapter.py',
        parameters=[{'use_sim_time': True}], condition=IfCondition(nav_ok), output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sampling_mode", default_value=defaults["sampling_mode"],
                                  choices=["snapshot", "micro_scan"]),
            DeclareLaunchArgument("micro_publish_mode", default_value=defaults["micro_publish_mode"],
                                  choices=["composite", "wedge40"]),
            DeclareLaunchArgument("micro_batches", default_value=defaults["micro_batches"],
                                  description="40 Hz acquisitions composited into 10 Hz Livox scans with four batches."),
            DeclareLaunchArgument("wheel_proportional_gain", default_value="0.30",
                                  description="Simulation wheel proportional gain; ordinary navigation default preserved"),
            DeclareLaunchArgument("wheel_integral_gain", default_value="1.0",
                                  description="Simulation wheel integral gain; default 1.0 preserves ordinary navigation"),
            DeclareLaunchArgument("spawn_pose", default_value="",
                                  description="Optional simulation spawn JSON [x,y,z,yaw]"),
            DeclareLaunchArgument(
                "scene", default_value="rmul_2027",
                choices=["rmul_2027", "tunnel_spec_250", "tunnel_spec_235"],
                description="rmul_2027 keeps Point-LIO/GICP; the drawing-size "
                            "fixtures require physics_test:=true and are not "
                            "localization acceptance. tunnel_spec_235 is the "
                            "negative clearance case only."),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument('render_engine', default_value=defaults['render_engine'], choices=['ogre2', 'ogre']),
            DeclareLaunchArgument('render_engine_library', default_value=defaults['render_engine_library']),
            DeclareLaunchArgument('real_time_factor', default_value=defaults['real_time_factor']),
            DeclareLaunchArgument("localization", default_value="true"),
            DeclareLaunchArgument('map_mode', default_value='prior', choices=['prior', 'mapping']),
            DeclareLaunchArgument(
                "physics_test", default_value="false",
                description="DIAGNOSTIC bounded mechanical test gate; never "
                            "publishes localization validity or map TF."),
            DeclareLaunchArgument('navigation', default_value=PythonExpression([
                "'", defaults['navigation'], "' if '", localization,
                "' == 'true' and '", physics_test, "' == 'false' and '", scene,
                "' == 'rmul_2027' else 'false'"])),
            DeclareLaunchArgument('self_filter', default_value='true'),
            DeclareLaunchArgument('saved_map_manifest', default_value=''),
            DeclareLaunchArgument('initial_imu_pose', default_value=''),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value="sim.rviz",
                                  choices=["sim.rviz", "raw_lidar.rviz"],
                                  description="Navigation view or raw sensor view independent of localization TF"),
            DeclareLaunchArgument(
                "robot_model",
                default_value="huananhu",
                choices=["huananhu", "low_profile_robot"],
            ),
            # proxy (MecanumDrive2) is kept for parity but cannot move this
            # composed model: its wrench cap (100 N) is below the ~310 N
            # friction of the CAD undercarriage resting on the field floor.
            DeclareLaunchArgument(
                "drive_model", default_value="rollers", choices=["proxy", "rollers"]
            ),
            DeclareLaunchArgument("robot_name", default_value="huananhu"),
            gazebo,
            clock_bridge,
            spawn,
            command_gate,
            physics_gate,
            posture_controller,
            lidar_adapter,
            point_lio,
            state_adapter,
            local_map_origin,
            TimerAction(period=8.0, actions=[gicp]),
            navigation_ready_handler,
            navigation_wait,
            nav_command_adapter,
            rviz_node,
        ]
    )
