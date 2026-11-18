#!/usr/bin/env bash
# SSH entry: mapping and prior-map navigation are mutually exclusive.
set -eo pipefail
mode="${1:-}"
if [[ "$mode" != mapping && "$mode" != navigation ]]; then
  echo 'Usage: bash scripts/real_robot.bash mapping|navigation'
  echo 'Environment: USE_RVIZ=false, NAV_PARAMS, NAV_MAP, GICP_CONFIG, FILTER_CONFIG'
  echo 'Sensor/LIO must already be running. Optional navigation: NAV_SERIAL=rmul_v1, NAV_SERIAL_PORT=/dev/ttyACM0'
  exit 2
fi
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo 'Missing install/setup.bash: install dependencies and build the workspace first.' >&2
  exit 1
fi
source "$workspace/install/setup.bash"
set -u
for config_env in NAV_PARAMS NAV_MAP GICP_CONFIG FILTER_CONFIG; do
  if [[ -n "${!config_env:-}" && ( "${!config_env}" != /* || ! -r "${!config_env}" ) ]]; then
    echo "$config_env must name a readable absolute file." >&2
    exit 1
  fi
done
case "${USE_RVIZ:-false}" in true|false) ;; *) echo 'USE_RVIZ must be true or false.' >&2; exit 2 ;; esac
serial_mode="${NAV_SERIAL:-off}"
case "$serial_mode" in
  off) ;;
  rmul_v1)
    if [[ "$mode" != navigation ]]; then
      echo 'NAV_SERIAL=rmul_v1 is only available in navigation mode.' >&2
      exit 2
    fi
    ;;
  *) echo 'NAV_SERIAL must be off or rmul_v1.' >&2; exit 2 ;;
esac
command -v tmux >/dev/null || { echo 'tmux is required.' >&2; exit 1; }
# Resolve installed resources before starting any processes.
packages=(navi2 point_lio livox_ros_driver2 rviz2 tf2_ros)
resources=(behavior_tree/navigate_to_pose_w_replanning_and_recovery.xml
           behavior_tree/navigate_through_pose_w_replanning_and_recovery.xml)
if [[ "$mode" == mapping ]]; then
  packages+=(slam_toolbox pointcloud_to_laserscan nav2_map_server)
  resources+=(scripts/savemap.bash params/slam.yaml config/slam.rviz)
else
  packages+=(icp_relocalization msg_convert minco_planner minco_controller
             nav2_bringup nav2_planner nav2_controller nav2_bt_navigator
             nav2_smoother nav2_behaviors nav2_waypoint_follower
             nav2_velocity_smoother nav2_lifecycle_manager spatio_temporal_voxel_layer)
  resources+=(params/sentry1.yaml config/our.rviz)
fi
if [[ "$serial_mode" == rmul_v1 ]]; then
  packages+=(communication)
fi
for package in "${packages[@]}"; do
  ros2 pkg prefix "$package" >/dev/null || {
    echo "Missing installed ROS package: $package" >&2; exit 1;
  }
done
nav_share="$(ros2 pkg prefix --share navi2)"
for resource in "${resources[@]}"; do
  [[ -r "$nav_share/$resource" ]] || {
    echo "Missing installed resource: $nav_share/$resource; rebuild with the updated install rules." >&2
    exit 1
  }
done
if [[ "$serial_mode" == rmul_v1 ]]; then
  comm_prefix="$(ros2 pkg prefix communication)"
  if [[ ! -r "$comm_prefix/share/communication/launch/basic_nav_serial.launch.py" ||
        ! -x "$comm_prefix/lib/communication/basic_nav_serial" ]]; then
    echo 'Missing installed RMUL serial launch or executable.' >&2
    exit 1
  fi
fi
session=ds_nav_real
# One session for both modes prevents this entry from starting duplicate map TF publishers.
if tmux has-session -t "$session" 2>/dev/null; then
  echo "Session $session already exists; inspect it with: tmux attach -t $session" >&2
  exit 1
fi
# Refuse known external map publishers too; inability to inspect is an error.
nodes="$(timeout 10s ros2 node list)" || exit 1
if [[ "$nodes" == *slam_toolbox* || "$nodes" == *gicp_relocalization_node* || "$nodes" == *map_server* ]]; then
  echo 'Existing mapping/localization nodes detected. Inspect and stop the old mode first.' >&2
  exit 1
fi
if [[ "$serial_mode" == rmul_v1 ]] &&
   [[ "$nodes" == *communication_sentry* || "$nodes" == *basic_nav_serial* ]]; then
  echo 'A chassis communication node is already running. Stop it before selecting RMUL V1.' >&2
  exit 1
fi
if [[ "$nodes" != *livox_pointlio_container* ]]; then
  echo 'Start the Point-LIO component container first (see README real-robot section).' >&2
  exit 1
fi
cd "$workspace"
# tmux servers may have been created before the ROS overlay was sourced.
# Explicitly load this workspace in every window instead of trusting server environment.
printf -v prefix 'cd %q && source /opt/ros/humble/setup.bash && source %q && ' "$workspace" "$workspace/install/setup.bash"
for ros_env in ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS MAP_OUTPUT_DIR DISPLAY XAUTHORITY; do
  if [[ -v "$ros_env" ]]; then
    printf -v env_value %q "${!ros_env}"
    prefix+="export ${ros_env}=${env_value} && "
  else
    prefix+="unset ${ros_env} && "
  fi
done
start_window() {
  local name="$1"; shift
  local command_text shell_command
  printf -v command_text '%q ' "$@"
  printf -v shell_command 'bash -c %q' "${prefix}exec ${command_text}"
  if ! tmux has-session -t "$session" 2>/dev/null; then
    tmux new-session -d -s "$session" -n "$name" "$shell_command"
    tmux set-option -t "$session" remain-on-exit on
  else
    tmux new-window -t "$session" -n "$name" "$shell_command"
  fi
}
if [[ "$mode" == mapping ]]; then
  start_window mapping ros2 launch navi2 slam.launch.py "use_rviz:=${USE_RVIZ:-false}"
else
  gicp=(ros2 launch icp_relocalization gicp_relocalization.launch.py)
  [[ -z "${GICP_CONFIG:-}" ]] || gicp+=("gicp_config:=$GICP_CONFIG")
  # Navigation launch supplies the local STVL filter; config may be overridden independently.
  filter=(ros2 launch msg_convert cloud_registered_crop_filter.launch.py)
  [[ -z "${FILTER_CONFIG:-}" ]] || filter+=("config_file:=$FILTER_CONFIG")
  nav=(ros2 launch navi2 navigation2.launch.py "use_rviz:=${USE_RVIZ:-false}" use_cloud_filter:=false)
  [[ -z "${NAV_PARAMS:-}" ]] || nav+=("params_file:=$NAV_PARAMS")
  [[ -z "${NAV_MAP:-}" ]] || nav+=("map:=$NAV_MAP")
  start_window localization "${gicp[@]}"
  start_window cloud_filter "${filter[@]}"
  start_window navigation "${nav[@]}"
  if [[ "$serial_mode" == rmul_v1 ]]; then
    start_window communication ros2 launch communication basic_nav_serial.launch.py \
      rmul_v1:=true "port:=${NAV_SERIAL_PORT:-/dev/ttyACM0}"
  fi
fi
echo "Started $mode. Inspect every window: tmux attach -t $session"
echo 'This is process startup only; verify localization and fresh inputs before sending goals.'
echo 'Before switching modes, stop each launch with Ctrl+C, verify nodes exited, then close the session.'
