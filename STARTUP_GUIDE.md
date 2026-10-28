# 启动说明

本文档说明当前自研导航栈的构建和启动方式。项目的仿真入口已经切换为
`rmu_gazebo_simulator`，本项目不再自带独立 Gazebo world/model。

## 1. 环境准备

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Gazebo Fortress
- 已安装 `colcon`、`rosdep`、`vcstool`

每个新终端先加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
```

如果是首次拉取 RMU 仿真依赖，先在工作空间根目录执行：

```bash
vcs import src < src/rmu_gazebo_simulator/dependencies.repos
```

`dependencies.repos` 会拉取完整 RMU 仿真需要的源码依赖：

- `sdformat_tools`
- `rmoss_interfaces`
- `rmoss_core`
- `rmoss_gazebo`
- `rmoss_gz_resources`
- `pb2025_robot_description`

其中 `rmoss_gazebo` 会提供多个运行包，包括：

- `rmoss_gz_base`
- `rmoss_gz_cam`
- `rmoss_gz_plugins`
- `rmoss_gz_bridge`

然后安装依赖：

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

RMU 仿真还依赖 Python 包 `xmacro`：

```bash
pip install xmacro
```

如果需要运行 RMU 的网页控制端，还需要安装网页端 Python 依赖：

```bash
pip install -r src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/player_web/requirement.txt
pip install flask-cors eventlet
sudo apt install python3-opencv python3-numpy
```

注意：仿真底盘控制至少需要 `rmoss_interfaces`、`rmoss_gz_base` 和 `rmoss_gz_bridge` 等 RMOSS 包。构建并 `source install/setup.bash` 后，可用下面命令检查：

```bash
for p in \
  rmoss_core rmoss_interfaces rmoss_gz_base rmoss_gz_cam \
  rmoss_gz_plugins rmoss_gz_resources rmoss_gz_bridge \
  pb2025_robot_description sdformat_tools
do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && echo "OK $p" || echo "MISSING $p"
done
```

如果缺少这些包，导航栈可能仍可启动到部分节点，但 `rmu_gazebo_simulator`
不能完整运行，`/cmd_vel` 也不会被可靠转发成 RMU 底盘命令。

## 2. 构建

在工作空间根目录执行：

```bash
colcon build --symlink-install
```

构建完成后加载工作空间：

```bash
source install/setup.bash
```

每次打开新终端都需要重新执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. 启动 RMU 仿真导航

启动完整仿真导航链路：

```bash
ros2 launch my_nav_bringup sim.launch.py
```

该命令会启动：

- `rmu_gazebo_simulator`
- `demo_map_server`
- `planner_node`
- `path_follower_node`
- `cmd_vel_to_chassis_cmd.py`

默认控制机器人为 `red_standard_robot1`。启动 Gazebo 后，需要点击 Gazebo 左下角橙红色的启动按钮，让仿真时间开始推进。

常用启动参数：

```bash
ros2 launch my_nav_bringup sim.launch.py robot_name:=red_standard_robot1
ros2 launch my_nav_bringup sim.launch.py goal_x:=1.0 goal_y:=2.0 goal_yaw:=0.0
ros2 launch my_nav_bringup sim.launch.py use_demo_map_server:=false
```

如果要切换 RMU 世界，修改：

```text
src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml
```

其中 `world` 当前默认是 `rmul_2026`。

## 4. 启动离线/准实车模式

不启动 Gazebo，仅启动导航核心和本地 fake 底盘：

```bash
ros2 launch my_nav_bringup reality.launch.py
```

默认参数会启用：

- `robot_description.launch.py`
- `demo_map_server`
- `planner_node`
- `path_follower_node`
- `fake_base_driver_node`
- 静态 `map -> odom` TF

该模式用于没有仿真或真实底盘时的离线验证。

常用参数：

```bash
ros2 launch my_nav_bringup reality.launch.py use_fake_hardware:=true
ros2 launch my_nav_bringup reality.launch.py goal_x:=2.8 goal_y:=2.8
```

## 5. 发送目标点

规划器默认目标为 `(2.8, 2.8)`。也可以手动发布 `/goal_pose`：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 2.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

## 6. 关键话题

导航侧常用话题：

- `/map`
- `/goal_pose`
- `/planned_path`
- `/cmd_vel`

RMU 仿真侧常用话题：

- `/<robot_name>/chassis_odometry_gt`
- `/<robot_name>/robot_base/chassis_cmd`
- `/<robot_name>/rplidar_a2/scan`
- `/<robot_name>/livox/lidar`
- `/<robot_name>/livox/imu`

默认仿真启动会把 `/cmd_vel` 转换为：

```text
/<robot_name>/robot_base/chassis_cmd
```

## 7. 快速检查

查看节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list
```

检查地图：

```bash
ros2 topic echo /map --once
```

检查规划路径：

```bash
ros2 topic echo /planned_path --once
```

检查 RMU 里程计：

```bash
ros2 topic echo /red_standard_robot1/chassis_odometry_gt --once
```

检查底盘命令是否输出：

```bash
ros2 topic echo /red_standard_robot1/robot_base/chassis_cmd
```

## 8. 常见问题

### 找不到 `rmu_gazebo_simulator`

确认已经构建并加载工作空间：

```bash
colcon build --symlink-install
source install/setup.bash
```

### 机器人不动

优先检查三点：

1. Gazebo 左下角启动按钮是否已点击。
2. `rmoss_interfaces` 是否存在。
3. 是否能看到 `/<robot_name>/robot_base/chassis_cmd` 上有消息。

### 一直等待 TF

仿真模式默认使用 `map -> odom` 静态 TF，并将控制基准 frame 设置为 `chassis`。如果日志提示等待 TF，检查：

```bash
ros2 run tf2_ros tf2_echo map chassis
```

### 规划失败

当前仍使用 `demo_map_server` 提供简化占据栅格。若 RMU 世界与 demo map 不匹配，可能出现路径穿模或规划失败。后续应将 `maps/simulation/` 中的正式地图接入。
