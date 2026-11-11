#!/usr/bin/env bash
# Single entry for the RMUL 2027 + 华南虎 simulation.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./scripts/huananhu_sim.bash [--headless] [--rviz] [--no-localization]
                                  [--model huananhu|low_profile_robot]
                                  [--drive proxy|rollers] [--sampling snapshot|micro_scan]
                                  [--scene rmul_2027|tunnel_spec_250|tunnel_spec_235]
                                  [--physics-test] [--check] [--check-install]

  --navigation        include Nav2 + MINCO + MPC (default)
  --no-navigation     localization / sensor diagnostics only
  --render-engine-library PATH  auto (default), or an explicit absolute library
  --map-mode MODE     prior (default) or mapping (no prior map)
  --render-engine ENG ogre (default) or ogre2 (diagnostic; CAD yaw failure on this GPU)
  --real-time-factor F simulator pacing only (default 0.2); physics step and sensor sim rates unchanged
  --saved-map FILE    load a validated saved-map manifest in prior mode
  --initial-imu-pose JSON  optional explicit map-to-initial-IMU [x,y,z,r,p,y]
  --headless          run Gazebo server only (-r -s)
  --rviz              start RViz
  --no-localization   start physics + bridges only, no Point-LIO/GICP
  --model NAME        default huananhu
  --drive MODE        default rollers (proxy is friction-pinned, see docs)
  --sampling MODE     snapshot (diagnostic) or micro_scan (default: 40 Hz / 4 batches)
  --scene NAME        rmul_2027 (default), tunnel_spec_250 or tunnel_spec_235.
                      Fixtures require --physics-test and never start GICP.
  --physics-test      DIAGNOSTIC bounded mechanical test gate. No localization
                      validity or map TF; mutually exclusive with the formal gate.
  --check             only verify the overlay provides the required packages
  --check-install     additionally verify the installed resource files
EOF
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws_dir="$(dirname -- "${script_dir}")"
defaults_file="${ws_dir}/src/simulation_integration/huananhu_sim_bringup/config/simulation_defaults.json"
read -r default_sampling default_navigation default_renderer default_factor < <(
  /usr/bin/python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(*(d[k] for k in ("sampling_mode","navigation","render_engine","real_time_factor")))' "${defaults_file}"
)

headless=false
rviz=false
localization=true
model=huananhu
drive=rollers
check_only=false
check_install=false
sampling="${default_sampling}"
scene=rmul_2027
physics_test=false
navigation="${default_navigation}"
map_mode=prior
render_engine="${default_renderer}"
real_time_factor="${default_factor}"
saved_map_manifest=
render_engine_library=auto
navigation_explicit=false
initial_imu_pose=

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    --no-localization) localization=false ;;
    --no-navigation) navigation=false; navigation_explicit=true ;;
    --model) model="${2:?missing model}"; shift ;;
    --drive) drive="${2:?missing drive mode}"; shift ;;
    --sampling) sampling="${2:?missing sampling mode}"; shift ;;
    --scene) scene="${2:?missing scene}"; shift ;;
    --physics-test) physics_test=true ;;
    --navigation) navigation=true; navigation_explicit=true ;;
    --map-mode) map_mode="${2:?missing map mode}"; shift ;;
    --render-engine-library) render_engine_library="${2:?missing library path or auto}"; shift ;;
    --render-engine) render_engine="${2:?missing renderer}"; shift ;;
    --real-time-factor) real_time_factor="${2:?missing factor}"; shift ;;
    --saved-map) saved_map_manifest="${2:?missing manifest}"; shift ;;
    --initial-imu-pose) initial_imu_pose="${2:?missing pose JSON}"; shift ;;
    --check) check_only=true ;;
    --check-install) check_only=true; check_install=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

# Diagnostics do not implicitly request the now-default navigation baseline.
if [[ "${navigation_explicit}" == false && ( "${localization}" == false || "${physics_test}" == true || "${scene}" != rmul_2027 ) ]]; then
  navigation=false
fi
case "$render_engine" in ogre|ogre2) ;; *) echo "Invalid renderer: $render_engine" >&2; exit 2 ;; esac
case "$map_mode" in prior|mapping) ;; *) echo "Invalid map mode: $map_mode" >&2; exit 2 ;; esac
if [[ "$navigation" == true && ( "$physics_test" == true || "$localization" == false || "$scene" != rmul_2027 ) ]]; then
  echo "Navigation requires rmul_2027 with localization and the formal gate." >&2; exit 2
fi

case "$sampling" in snapshot|micro_scan) ;; *) echo "Invalid sampling mode: $sampling" >&2; exit 2 ;; esac
case "$scene" in
  rmul_2027) ;;
  tunnel_spec_250|tunnel_spec_235)
    # The fixture worlds have no matching prior map; GICP must not run.
    localization=false
    if [[ "${physics_test}" != true ]]; then
      echo "Scene ${scene} requires --physics-test (diagnostic gate, no localization)." >&2
      exit 2
    fi
    ;;
  *) echo "Invalid scene: $scene" >&2; exit 2 ;;
esac


if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble." >&2
  exit 1
fi
if [[ ! -f "${ws_dir}/install/setup.bash" ]]; then
  echo "Workspace overlay missing: build the workspace first (${ws_dir}/install/setup.bash)." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ws_dir}/install/setup.bash"
set -u

required_packages=(
  huananhu_sim_resources
  huananhu_sim_plugins
  huananhu_sim_bridge
  huananhu_sim_bringup
  ros_gz_sim
  ros_gz_bridge
  point_lio
  icp_relocalization
)
if [[ "${navigation}" == true ]]; then
  required_packages+=(nav2_planner nav2_controller nav2_bt_navigator nav2_waypoint_follower nav2_map_server nav2_lifecycle_manager minco_planner minco_controller)
fi
for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "Required package missing from overlay: ${package}" >&2
    exit 1
  fi
done

installed_defaults="$(ros2 pkg prefix huananhu_sim_bringup)/share/huananhu_sim_bringup/config/simulation_defaults.json"
if ! cmp -s "${defaults_file}" "${installed_defaults}"; then
  echo "Installed simulation defaults are stale; rebuild huananhu_sim_bringup." >&2; exit 1
fi
for executable in sim_lidar_to_livox.py sim_state_adapter.py sim_command_gate.py sim_posture_controller.py sim_physics_test_gate.py sim_nav_command_adapter.py; do
  if [[ ! -x "$(ros2 pkg prefix huananhu_sim_bridge)/lib/huananhu_sim_bridge/${executable}" ]]; then
    echo "Installed bridge executable missing or not executable: ${executable}" >&2; exit 1
  fi
done

if [[ "${navigation}" == true && ! -x "$(ros2 pkg prefix huananhu_sim_bringup)/lib/huananhu_sim_bringup/simulation_planner_server" ]]; then
  echo "Installed simulation_planner_server missing; rebuild huananhu_sim_bringup." >&2; exit 1
fi
if [[ "${render_engine}" == ogre && "${render_engine_library}" == auto ]]; then
  /usr/bin/python3 - "$(ros2 pkg prefix huananhu_sim_bringup)/share/huananhu_sim_bringup/runtime/rendering" <<'PYRENDER'
import hashlib, json, sys
from pathlib import Path
runtime = Path(sys.argv[1])
for name, expected in json.loads((runtime / 'SHA256.json').read_text()).items():
    if hashlib.sha256((runtime / name).read_bytes()).hexdigest() != expected:
        raise SystemExit('Packaged renderer checksum mismatch: ' + name)
PYRENDER
fi
if [[ "${check_install}" == true ]]; then
  res_share="$(ros2 pkg prefix huananhu_sim_resources)/share/huananhu_sim_resources"
  for path in \
    "config/tunnel_profile.json" \
    "config/cad_support_vertices.json" \
    "config/linkage_geometry.json" \
    "scripts/kinematic_math.py" \
    "worlds/${scene}_world.sdf"; do
    if [[ ! -f "${res_share}/${path}" ]]; then
      echo "Installed resource missing: ${res_share}/${path}" >&2
      exit 1
    fi
  done
  hulls=$(find "${res_share}/models/huananhu_demo/collision" -name '*_hull.obj' 2>/dev/null | wc -l)
  if [[ "${hulls}" -ne 14 ]]; then
    echo "Installed CAD hulls: ${hulls}, expected 14 (rebuild with --symlink-install)." >&2
    exit 1
  fi
fi

if [[ "${check_only}" == true ]]; then
  echo "Preflight passed: ${model}, drive=${drive}, scene=${scene}, physics_test=${physics_test}, navigation=${navigation}, sampling=${sampling}, localization=${localization}, headless=${headless}."
  exit 0
fi

# Keep this workspace separate from other local robots/simulators by default.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export IGN_PARTITION="${IGN_PARTITION:-ds_nav_sim}"
export GZ_PARTITION="${GZ_PARTITION:-${IGN_PARTITION}}"
exec 9>"${TMPDIR:-/tmp}/ds_nav_sim_${UID}_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
  echo "A ds_nav simulation already owns domain ${ROS_DOMAIN_ID}." >&2
  exit 1
fi
echo "Starting ds_nav: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, IGN_PARTITION=${IGN_PARTITION}, scene=${scene}, sampling=${sampling}, physics_test=${physics_test}"
extra_launch_args=()
if [[ -n "${saved_map_manifest}" ]]; then extra_launch_args+=("saved_map_manifest:=${saved_map_manifest}"); fi
if [[ -n "${initial_imu_pose}" ]]; then extra_launch_args+=("initial_imu_pose:=${initial_imu_pose}"); fi
exec /usr/bin/python3 "${script_dir}/sim_supervisor.py" -- ros2 launch huananhu_sim_bringup simulation.launch.py \
  headless:="${headless}" \
  rviz:="${rviz}" \
  localization:="${localization}" \
  physics_test:="${physics_test}" \
  scene:="${scene}" \
  robot_model:="${model}" \
  drive_model:="${drive}" \
  sampling_mode:="${sampling}" \
  navigation:="${navigation}" \
  map_mode:="${map_mode}" \
  render_engine:="${render_engine}" \
  render_engine_library:="${render_engine_library}" \
  real_time_factor:="${real_time_factor}" \
  "${extra_launch_args[@]}"
