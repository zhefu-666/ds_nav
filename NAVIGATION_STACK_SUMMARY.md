# 脱离 Nav2 的自研导航栈总结

## 1. 本地对比了什么

已在本地对比 `Dust_sentry_navigation`，重点看了：

- `pb2025_nav_bringup/launch/rm_navigation_simulation_launch.py`
- `pb2025_nav_bringup/launch/rm_navigation_reality_launch.py`
- `pb2025_nav_bringup/launch/bringup_launch.py`
- `pb2025_robot_description/launch/robot_description_launch.py`
- `pb2025_nav_bringup/map/simulation/`
- `pb2025_nav_bringup/pcd/simulation/`

Dust 的核心不是单个规划器，而是把 `robot_description`、地图、仿真、实车四条线分开。

## 2. 当前仓库已经具备什么

- `my_global_planner`：A* 全局规划器
- `planner_node`：订阅 `/map`、`/odom`、`/goal_pose`，发布 `/planned_path`
- `demo_map_server`：可运行的占据栅格地图发布器
- `my_nav_control`：共享控制层
  - `path_follower_node`：跟踪规划路径并发布 `/cmd_vel`
  - `fake_base_driver_node`：无实车时模拟 `/odom` 和 `odom -> base_link`
- `my_nav_bringup`：双入口 bringup
  - `sim.launch.py`
  - `reality.launch.py`
  - `navigation_core.launch.py`
  - `robot_description.launch.py`

## 3. 现在的运行逻辑

### 仿真

- `rmu_gazebo_simulator` 负责世界、机器人和裁判系统
- `demo_map_server` 提供地图
- `planner_node` 输出路径
- `path_follower_node` 把路径变成 `/cmd_vel`
- `cmd_vel_to_chassis_cmd.py` 把 `/cmd_vel` 适配到 `/<robot_name>/robot_base/chassis_cmd`
- RMU 仿真里的机器人底盘读取 `ChassisCmd`，里程计通过 `/<robot_name>/chassis_odometry_gt` 进入规划链路

### 实车/准实车

- 不启动 Gazebo
- 仍然保留同一套 `/map`、`/planned_path`、`/cmd_vel` 接口
- 先用 `fake_base_driver_node` 做离线验证
- 以后把 fake driver 换成串口/UDP/底盘驱动即可

## 4. 还需要完成什么

### 感知和定位

- 雷达驱动
- IMU 接入
- 点云预处理
- 定位/TF 链路
- map -> odom 的真实维护

### 地图

- 把当前 demo map 替换成真正的静态地图加载流程
- 按 Dust 风格补齐 `maps/simulation/`、`maps/reality/`、`pcd/simulation/`、`pcd/reality/`

### 控制

- 将 `fake_base_driver_node` 替换为真实底盘接口
- 补速度限制、急停、超时保护、通信状态检测

### 工程化

- RMU 仿真地图与导航地图的正式对齐
- 更细的命令接口适配和急停策略
- RViz 配置
- 参数文件拆分
- 单元测试和 launch 检查
- 更完整的回归验证

## 5. 推荐下一步顺序

1. 先让 RMU 仿真链路稳定跑起来
2. 再把静态地图文件接入，而不是只靠 demo map server
3. 再接真实定位链路
4. 最后替换 fake base driver 为实车驱动

## 6. 结论

当前仓库已经从“只有 A*”变成了“能接 RMU 仿真的自研导航骨架”。

下一阶段的重点不是再堆 Nav2，而是把：

- 地图
- 定位
- 控制后端
- 实车驱动

这四块补成和 Dust 类似的工程化结构。
