# ds_nav 仿真使用与调试说明

核对日期：2026-11-10。工作空间：`/home/liu/ds_nav`。

本文依据当前启动脚本、launch、源码、参数和已有运行产物整理；编写时没有启动新仿真或执行构建。当前进度以第1节列出的实际结果为准，历史交接文档中的“运行中”“东侧尚未五次通过”等描述可能已经过时。

## 归档后的路径说明

2026-11-10已将历史实验和报告移出项目，完整归档位于`/home/liu/ds_nav_archive/20261110_cleanup/`，清理manifest位于该目录`cleanup_manifest.json`。本文历史`artifacts/...`目录均指归档下同名路径，历史报告链接已更新。归档保留评分、失败原始数据、地图包和实验脚本。

归档实验脚本含旧工作空间绝对路径，仅用于查阅和恢复复现；不要直接从归档目录运行旧队列。若要重现，应先按manifest恢复所需文件与依赖，使用新输出目录并检查脚本路径。正常启动和scripts中的通用工具仍在项目内；`check_sim_clearance.py`运行时可重新创建其默认输出目录，这不代表旧产物未清理。

## 1. 当前完成情况

**已知地图、两处已知隧道、显式任务入口下，9项主场景和2项额外冷启动复验已经全部通过。** 这不代表所有仿真能力、所有故障模式或实车已经验收。

最终配置运行记录位于：

- [主场景状态](/home/liu/ds_nav_archive/20261110_cleanup/artifacts/final_acceptance_20261103/yaw_margin_v2/state.json)：`PASS_DEFINED_SCOPE`，9项。
- [冷启动复验状态](/home/liu/ds_nav_archive/20261110_cleanup/artifacts/final_acceptance_20261103/yaw_margin_v2/repeatability_state.json)：`PASS_DEFINED_SCOPE`，2项。
- [自动回归日志](/home/liu/ds_nav_archive/20261110_cleanup/artifacts/final_acceptance_20261103/yaw_margin_v2/combined_regression.log)：127项通过。
- [本轮调试与失败记录](/home/liu/ds_nav_archive/20261110_cleanup/docs/ai_refactor_records/20261103_final_simulation_acceptance.md)：保存修补过程；其中早期的“运行中”状态不替代上述最终状态文件。

| 已验证项目 | 实际结果 | 原始产物目录，均位于 `artifacts/tunnel_integration_20261102/` |
|---|---|---|
| 普通场地导航 | 30/30，最大真实终点65.03mm，最大yaw误差2.1885°，运动位置RMSE20.49mm；取消停车通过 | `joint_acceptance_30goals_yawmargin_20261103_01` |
| 无先验建图导航及保存 | 4/4，RMSE10.40mm；地图包有效 | `joint_acceptance_mapping_yawmargin_20261103_01` |
| 加载刚保存地图后导航 | 3/3，RMSE6.11mm；取消停车通过 | `joint_acceptance_reload_yawmargin_20261103_01` |
| 西侧完整任务 | 五次交替方向穿行＋移出＋GICP＋导航重建＋最终目标；最大穿行终点26.33mm，零接触 | `west_five_diagnostic_yawmargin_20261103_01` |
| 东侧完整任务 | 同上，最大穿行终点20.82mm，零接触 | `east_five_diagnostic_yawmargin_20261103_01` |
| 双侧洞内取消 | 两侧独立通过，disabled确认，清理后命令全零 | `east_passage_yawmargin_20261103_01`、`west_passage_yawmargin_20261103_01` |
| 双侧洞内Point-LIO退出 | 两侧独立通过，零接触、停车、命令归零 | `east_odom_failure_yawmargin_20261103_01`、`west_odom_failure_yawmargin_20261103_01` |
| 双侧额外冷启动 | 各一次完整任务通过；最终目标误差西24.68mm、东7.89mm | `west_transition_diagnostic_yawmargin_20261103_01`、`east_transition_diagnostic_yawmargin_20261103_01` |

五次穿行的最小真实顶部/侧向余量：西侧4.58/12.73mm，东侧4.51/10.87mm。它们是完整CAD包络在独立评估采样中的结果，不应以名义车高240mm替代。

本次核对主套件锁定的450个源码/资源文件，内容哈希与当前文件全部一致；没有发现匹配验收队列或Gazebo启动命令的运行进程。这是本次进程快照，不是永久运行保证。

### 仍有边界与问题

- 历史上重定位后发生过SMAC搜索耗尽；最终双侧五次及额外冷启动没有重现，但根因尚未被独立确认。
- 最终套件的建图轮、东侧取消轮仍出现过Gazebo退出升级到SIGKILL；任务/停车评分通过不等于无异常退出。
- 取消评分器没有把零接触纳入自动PASS条件；无残留进程、隧道评估全程覆盖率也尚未形成统一自动门槛。
- 未覆盖未知隧道发现、任意路线选择、动态障碍专项矩阵、坡面/台阶、长期压力、网络故障、多算法和实车验收。
- `state_contract.yaml`仍保留早期“运行未验证”等描述；其中传感器安装假设、协方差未传播、动态yaw1运动未完整验收等限制不能因折叠定姿态通行通过而自动消除。

## 2. 系统实际运行链路

```text
RMUL 2027 CAD场地 + 真实车体碰撞/轮驱动 + Gazebo Fortress
  ├─ /lidar/points → CAD自过滤/扫描时序桥 → /livox/lidar
  ├─ /livox/imu → Point-LIO → /aft_mapped_to_init、/cloud_registered_full
  └─ /joint_states → 状态恢复、姿态就绪
                         ↓
          状态适配 /sim/base_odom + 定位有效性
                         ↓
      Nav2 → MINCO/ROGMap → /opt_path → MPC
                         ↓
      /sim/nav_cmd_vel_global → 坐标转换 → /sim/auto_cmd_vel
                         ↓
             唯一正式 sim_command_gate
                         ↓
           /sim/actuator_cmd_vel → Gazebo轮驱动
```

`/sim/manual_cmd_vel`也只能经门控进入执行器。`/sim/ground_truth_odometry`和`/sim/wheel_odometry`是诊断/评估通道，不能作为正式定位或控制输入。

主要TF关系是 `map → camera_init → base_footprint → chassis → CAD关节/传感器`。先验模式的`map → camera_init`来自GICP；mapping模式使用局部初始世界系，并由启动文件发布单位变换。Point-LIO配置`tf_send_en: false`，由状态适配节点恢复导航基座TF；不要另加同边发布者。

仿真使用独立ROS进程；不要把实车component container和其参数文件当成仿真活动链路。`simulation_planner_server`内部承载官方Nav2 PlannerServer，MINCO创建并使用ROGMap，ROGMap不是本配置中的独立导航节点。

## 3. 环境与最短启动步骤

已配置环境使用ROS 2 Humble、Gazebo Fortress（`ign gazebo --force-version 6`）、`ros_gz_sim`/`ros_gz_bridge`及本工作空间overlay。需要图形渲染能力；`--headless`只是不显示GUI，不等于GPU LiDAR不需要渲染。

脚本要求已有`install/setup.bash`、已安装ROS包和随包校验的Ogre修补库。如果缺失，应先说明缺项并取得构建许可；不要绕过检查或直接覆盖系统渲染库。

终端A：

```bash
cd /home/liu/ds_nav
./scripts/huananhu_sim.bash --headless --check-install
./scripts/huananhu_sim.bash --headless
```

第一条只检查，不启动物理场景。第二条默认启动华南虎真实滚子轮模型、先验定位和导航；**定位就绪后才创建Nav2，不自动发送目标**。

需要Gazebo界面时省略`--headless`；需要RViz时增加`--rviz`：

```bash
./scripts/huananhu_sim.bash --rviz
```

终端B及之后每个ROS调试终端均执行：

```bash
cd /home/liu/ds_nav
source /opt/ros/humble/setup.bash
source /home/liu/ds_nav/install/setup.bash
export ROS_DOMAIN_ID=87
export IGN_PARTITION=ds_nav_sim
export GZ_PARTITION=ds_nav_sim
```

87和`ds_nav_sim`是统一入口在环境未设置时的默认值。如果启动终端显式改过domain/partition，调试终端必须一致。不要把专项验收的196/197/198与普通入口87混用。

单机网络诊断可在启动前、所有相关终端一致设置：

```bash
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
export GZ_IP=127.0.0.1
```

### 默认入口与验收配置的区别

| 项目 | 普通统一入口 | 最终专项验收 |
|---|---|---|
| 仿真实时因子 | 0.2 | 显式0.15 |
| 滚子轮P/I | 0.30 / 1.0 | 显式1.0 / 0.2 |
| 3D LiDAR | 40Hz采集，4批合成约10Hz | 同一采样模式 |
| 渲染器 | 包内修补Ogre，自动SHA256校验 | 同左 |
| 机器人初始位置 | `world.yaml`中`[-5,0,0.08,0]` | 普通任务默认位置；隧道任务按侧设置 |
| domain | 默认87 | 西196、东197、普通联合198 |

**专项参数通过不代表默认轮PID也已通过同一隧道任务。** Shell没有暴露轮PID和`spawn_pose`选项；这类覆盖需用ROS launch。直接launch会绕过Shell的domain默认值和文件锁，因此必须自己确保实例隔离、环境已source，并使用监督器。

## 4. 导航、建图和地图重载

### 4.1 发送普通导航目标

在终端A保持运行、终端B完成上述环境准备后：

```bash
python3 scripts/sim_nav_trial.py --goal -4.4 0 0 --out /tmp/ds_nav_goal.json
```

目标是当前`map`系的`x y yaw`，米和弧度。工具默认请求折叠`tunnel`姿态，等待定位、姿态和Nav2就绪；默认ready/goal超时各180墙钟秒。`--posture stand`只适用于明确允许站姿的场景，不能用来证明隧道可通行。

顺序任务文件是JSON数组，例如`[[-4.4,0,0],[-4.4,-1,0]]`：

```bash
python3 scripts/sim_nav_trial.py --mission /absolute/path/mission.json --out /tmp/ds_nav_mission.json
python3 scripts/sim_nav_trial.py --goal -4.4 -1 0 --cancel-after 8 --out /tmp/ds_nav_cancel.json
```

`--cancel-after`单位是墙钟秒。`completed`只表示任务动作完成；`cancel_test_passed`只表示取消确认、终态和disabled，不替代真值精度/物理停车评分。

### 4.2 无先验建图

结束上一实例后在终端A启动：

```bash
./scripts/huananhu_sim.bash --headless --map-mode mapping
```

该模式不启动GICP/先验map server；导航launch将MINCO切为EXPLORATION，移除静态costmap层。先等待局部初始化有效，再在终端B发目标。它是已验证的有限目标建图导航，不是自动探索全场。

### 4.3 保存并重载地图

mapping有效且已有实际观测后，在终端B执行：

```bash
python3 scripts/sim_map_bundle.py --duration 15 \
  --out /tmp/ds_nav_saved_map \
  --grid-started-after-valid --reload-seed 0 0 0 0 0 0
```

输出目录必须不存在。`--grid-started-after-valid`是调用者对启动时序的声明，不是强制通过开关；必须确实在mapping有效之后创建新鲜ROGMap。示例reload seed仅适用于回到相应局部初始IMU姿态的场景，不能拿它替代任意新出生点的真实初始位姿。地图包包括PCD、栅格YAML/图像、哈希和manifest，必须检查其`status`为`VALID_MAP_BUNDLE`。

结束建图实例后：

```bash
./scripts/huananhu_sim.bash --headless --saved-map /tmp/ds_nav_saved_map/manifest.json
```

重载验证manifest、坐标约定、文件哈希和YAML图像对应关系。新地图的`map`属于该次局部建图坐标约定，不是自动恢复为原赛场全局坐标。

## 5. 隧道任务与机械诊断

### 5.1 显式已知隧道任务

`sim_tunnel_mission.py`按目标与已知隧道位置选择路线，串联普通Nav2、对正、局部通行、洞外移出、真实GICP、导航RESET/STARTUP和最终目标。它没有拦截普通`/navigate_to_pose`，不是未知隧道发现插件。

以下为已验收东侧专项启动配置。确认没有另一实例占用domain197，在终端A source环境并设置：

```bash
cd /home/liu/ds_nav
source /opt/ros/humble/setup.bash
source /home/liu/ds_nav/install/setup.bash
export ROS_DOMAIN_ID=197
export IGN_PARTITION=ds_nav_mission_197
export GZ_PARTITION=ds_nav_mission_197
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
python3 scripts/sim_supervisor.py -- ros2 launch huananhu_sim_bringup simulation.launch.py \
  headless:=true real_time_factor:=0.15 \
  wheel_proportional_gain:=1.0 wheel_integral_gain:=0.2 \
  'spawn_pose:=[0.1,-1.5,0.08,0.0]' \
  'initial_imu_pose:=[0.15575971900513,-1.663594690492484,0.278643863403111,0,0,0]'
```

终端B source相同overlay、设置相同domain/partition/回环环境后：

```bash
python3 scripts/sim_tunnel_mission.py --goal 4.8 -2 0 --passes 1 --out /tmp/ds_nav_east_mission
```

五次重复用`--passes 5`并换一个不存在的输出目录；含义是五次交替方向穿行。支持奇数1/3/5/7/9，保证最后停在目标一侧。西侧验收出生点为`[-4.8,3.5,0.08,0]`，IMU初值为`[-4.74424028099487,3.336405309507516,0.278643863403111,0,0,0]`，最终目标为`[-0.3,4,0]`，使用domain196及对应partition。

`spawn_pose`是`[x,y,z,yaw]`；`initial_imu_pose`是`[x,y,z,roll,pitch,yaw]`，二者不能直接照抄。先验模式改出生点时必须提供匹配IMU初值。初始化先验是配置，不是在线订阅真值。

`sim_tunnel_passage.py`是需要预先到达洞外对正点的局部工具，不宜从任意出生点直接启动。其CLI默认速度0.08m/s；完整任务内部使用的专项速度等配置应以任务源码为准，不与直接CLI混称。

### 5.2 机械夹具模式

```bash
./scripts/huananhu_sim.bash --headless --scene tunnel_spec_250 --physics-test
# 235mm拒绝对照场景：结束上一实例后再启动
./scripts/huananhu_sim.bash --headless --scene tunnel_spec_235 --physics-test
```

这两个命令只启动夹具诊断，不自动发车。夹具不运行GICP，使用独立physics-test gate，与正式定位门控互斥。不能用夹具通过替代正式自主导航验收；`--no-localization`也不意味着正式模式允许运动。

离线CAD/连杆检查，不启动Gazebo、不构建：

```bash
python3 scripts/check_sim_clearance.py --output /tmp/ds_nav_clearance.json
```

实际折叠CAD约244.47mm高、752.58mm宽；普通MINCO的圆形碰撞距离为0.602m，不能简单缩圆宣称通过约800mm宽隧道。专项用真实完整包络、已知结构观测和原净空保护。

## 6. 常用只读调试命令

先在调试终端source环境并匹配domain。不同终端分别运行持续观察命令：

```bash
ros2 node list
ros2 action list
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic echo /sim/localization_status --once
ros2 topic echo /sim/localization_valid --once
ros2 topic echo /sim/posture_ready --once
ros2 topic echo /sim/control_status --once
ros2 topic hz /lidar/points
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered_full
ros2 topic hz /sim/base_odom
ros2 topic info /sim/actuator_cmd_vel --verbose
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init base_footprint
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
```

话题频率受RTF和观察工具调度影响；区分仿真时间频率与墙钟接收速率。大点云和odom有best-effort/latest-state QoS，发现“无消息”时先检查QoS和domain，不要先改频率。

| 话题 | 作用 |
|---|---|
| `/clock` | 仿真时钟 |
| `/lidar/points`、`/livox/lidar` | 渲染点云与送入Point-LIO的Livox格式点云 |
| `/aft_mapped_to_init`、`/cloud_registered_full` | 原始LIO状态、完整去畸变点云 |
| `/sim/base_odom`、`/sim/lidar_odom` | 恢复后的基座与雷达原点状态 |
| `/sim/localization_valid`、`/sim/localization_status` | 定位门控及原因 |
| `/sim/control_mode`、`/sim/control_status` | 请求模式与实际执行状态 |
| `/sim/auto_cmd_vel`、`/sim/manual_cmd_vel` | 门控输入；不要绕到执行器输出话题发速度 |
| `/sim/actuator_cmd_vel` | 唯一正式门控输出，排查停车时观察此处 |
| `/huananhu/posture`、`/sim/posture_ready` | 姿态请求与实际关节就绪 |
| `/opt_path` | MINCO到MPC的专用轨迹接口，不等于任意nav_msgs/Path |
| `/rog_map/layer_value{,_static,_dynamic}`、`/rog_map/field` | 融合/静态/在线投影与距离场 |
| `/assessment/body_contacts` | 车体接触记录 |
| `/sim/ground_truth_odometry` | 独立评分真值，正式控制禁止消费 |

需要急停时，在匹配domain的终端发布模式，并确认control_status及执行器速度：

```bash
ros2 topic pub --once /sim/control_mode std_msgs/msg/String '{data: estop}'
```

正式门控默认disabled，manual/auto互斥；停止/模式切换会清除旧命令。正常停止在任务终端和启动终端按Ctrl+C，先让任务取消/禁用，再让监督器清理其拥有的进程。不要使用广泛`pkill`，也不要删除锁文件绕过仍运行的实例。

## 7. 采集、TF检查和独立评分

在另外终端执行采集；`--duration`为最大墙钟秒：

```bash
python3 scripts/sim_sensor_probe.py --seconds 20 --out /tmp/ds_nav_sensor_probe.json
python3 scripts/sim_tf_validate.py --duration 15 --out /tmp/ds_nav_tf.json
python3 scripts/sim_trajectory_recorder.py --mode prior --duration 180 --out /tmp/ds_nav_trace.jsonl
# 待记录器结束，再评分
python3 scripts/sim_trajectory_evaluate.py /tmp/ds_nav_trace.jsonl --out /tmp/ds_nav_score.json
```

建图采集用`--mode mapping`，保存地图重载采集用`--mode saved-local`。TF工具检查拓扑/时间等，不证明发布者唯一。`SCORED`表示记录具备运动评分条件，不等于目标任务通过。

正式验收还需结合任务结果、终点真值、接触、净空和停车：

- 普通导航：严格30/30、各目标真实位置误差<0.15m、yaw≤3°，运动位置RMSE≤0.10m、yaw RMSE≤3°，无有效接触。
- 隧道：穿行终点位置<0.05m、yaw<0.035rad；入口/移出位置<0.05m，普通接近/最终目标位置<0.15m；实际包络余量为正、零接触、disabled确认。
- 取消/故障：允许2仿真秒稳定，再至少观察1秒；速度≤0.05m/s、角速度≤0.05rad/s，清理后命令全零。洞内试验还必须证明触发前存在运动、实际位置位于洞内。

`artifacts/p1_navigation_20261101/assess_mission.py`旧基础评分器允许29/30及0.20m；最终队列另加严格检查，不能单看其`passed=true`。曾有30个动作成功但真实yaw3.0665°的失败，已保留原始结果；内部朝向完成条件因此收紧到2.5°，没有放宽独立3°评分。

现有验收脚本用于复现历史实验，含固定domain、目录及专项PID，不是通用任务管理器：

| 脚本 | 用途 |
|---|---|
| `artifacts/tunnel_integration_20261102/run_joint.py` | 自有仿真实例：普通30目标、mapping、reload |
| `artifacts/tunnel_integration_20261102/run_mission.py` | 自有双侧隧道实例、取消及明确的Point-LIO退出注入 |
| `artifacts/tunnel_integration_20261102/assess_mission.py` | 独立完整任务终点、接触、CAD净空 |
| 同目录 `assess_mission_cancel.py`、`assess_mission_failure.py` | 洞内取消/故障物理停车 |
| `artifacts/final_acceptance_20261103/yaw_margin_v2/run_final.py` | 已结束的9项主队列，锁定源码版本 |
| 同目录 `repeat_transitions.py` | 主队列通过后双侧冷启动复验 |

不要直接重启已完成队列覆盖目录或为了“同版本”一律全部重跑。参数/代码变化后按影响范围补测，保留未受影响的旧证据及其哈希；先分清动作完成、局部通过与完整任务通过。

## 8. 代码结构与参数来源

```text
scripts/
  huananhu_sim.bash                 统一启动、检查、隔离及锁
  sim_supervisor.py                自有进程与异常退出清理
  sim_nav_trial.py                 普通目标/顺序任务/取消
  sim_tunnel_mission.py            显式完整隧道任务
  sim_tunnel_passage.py            局部观测、桥接、包络与通行
  sim_map_bundle.py                地图保存/manifest
  sim_sensor_probe.py、sim_tf_validate.py
  sim_trajectory_recorder.py、sim_trajectory_evaluate.py
  check_sim_clearance.py           离线CAD与连杆闭合
src/simulation_integration/
  huananhu_sim_bringup/
    launch/                        simulation/navigation/spawn入口
    config/                        仿真参数
    maps/                          PCD、PGM、YAML及来源manifest
    behavior_tree/                 仿真Nav2树
    rviz/、runtime/rendering/        可视化及包内Ogre修补库
  huananhu_sim_bridge/
    scripts/                       传感器桥、状态恢复、姿态及门控
    test/、tests/                  离线回归与诊断工具
  huananhu_sim_resources/
    worlds/、models/、urdf/          场地、车体、碰撞及传感器
    config/                        CAD支撑点、折叠姿态、连杆几何
    scripts/                       组合模型、运动学和滚子轮生成
  huananhu_sim_plugins/plugins/     WheelTractionDrive、MecanumDrive2
src/perception/Point-LIO/           共用滤波与去畸变核心
src/perception/icp_relocalization/  共用GICP及仿真显式种子召回
src/perception/rog_map/             占据、投影、先验融合与ESDF
src/navigation/minco_planner/       Nav2 planner、SMAC与MINCO
src/navigation/minco_controller/    MPC及/opt_path消费
```

下表路径相对于`src/simulation_integration/`。文档中的参数不覆盖源码/启动override：

| 文件 | 主要职责及当前值 |
|---|---|
| `huananhu_sim_bringup/config/simulation_defaults.json` | Shell/launch共享默认：micro_scan、composite、4批、导航true、Ogre、RTF0.2 |
| 同目录 `world.yaml` | 场景与默认出生位姿；`spawn_pose`可覆盖 |
| 同目录 `bridge.yaml` | ROS↔Gazebo类型、方向与topic；含真值诊断隔离 |
| 同目录 `pointlio_sim.yaml` | `/livox/lidar`、`/livox/imu`；acc_norm9.8、satu_acc30、lidar_meas_cov0.01、ivox0.5m；odom100Hz；tf_send_en=false |
| 同目录 `pointlio_carrier.yaml` | 简化机器人诊断配置，不能与华南虎配置混用 |
| 同目录 `gicp_sim.yaml` | 3D定位、3帧累积、匹配5Hz、TF100Hz；收敛3次；显式参数种子召回true；PCD和IMU初值由launch覆盖 |
| 同目录 `navigation_sim.yaml` | 控制20Hz；vx/vy上限0.3m/s、omega0.5rad/s；内部yaw2.5°、xy0.15m；StoppedGoalChecker；respect_goal_yaw=true |
| 同上：ROGMap/MINCO子树 | 分辨率0.05m，scan_z范围[-0.2,1.5]；safe_dist0.61、collision_dist0.602；prior/mapping由launch改写 |
| 同目录 `state_contract.yaml` | 状态恢复约定及限制，非运行参数加载入口；其中部分状态标签仍为早期记录 |
| `huananhu_sim_resources/config/model.json` | 模型和机构参数 |
| 同目录 `tunnel_profile.json` | fold/yaw1/yaw2=0、aim4°；关节就绪容差0.01rad；CAD真实尺寸 |
| 同目录 `cad_support_vertices.json`、`linkage_geometry.json` | 完整包络与连杆几何来源 |

门控限制在`huananhu_sim_bridge/scripts/sim_command_gate.py`中硬编码：平移范数0.3m/s、yaw0.5rad/s，平移加速度0.3m/s²、角加速度0.8rad/s²；命令0.3墙钟秒超时、定位和主要传感器0.5秒新鲜度，倾斜0.35rad，接触后至少0.35秒停止。姿态联锁只在已有且新鲜的姿态就绪消息表示false时触发，不能据此声称姿态话题缺失也一定拒绝运动。

资源/参数解析顺序：统一脚本读源码defaults并比对安装defaults→ROS包路径解析→simulation.launch加载Point-LIO/GICP、桥和场景→定位就绪→navigation.launch读取导航YAML并生成临时参数文件。**检查实际安装路径是否symlink或旧副本；改源码不自动证明运行进程已重载。**

## 9. 常见故障定位

| 表现 | 优先检查 |
|---|---|
| 找不到话题/节点 | 是否source、domain一致、节点是否已创建；导航要等定位就绪 |
| `localization_invalid_or_stale` | `/sim/localization_status`、LIO/IMU/关节时间戳、GICP、TF和仿真是否暂停；不能关门控强行运动 |
| 姿态未就绪 | `/sim/posture_ready`和真实joint_states；请求字符串不等于关节到位 |
| `command_timeout` | 输入话题、模式和消息持续性；单条速度命令不会无限保持 |
| SMAC起终点free但搜不到路 | 同时检查ROGMap融合ESDF、静态/在线投影及重定位后的坐标关系；free栅格不等于满足0.602m距离 |
| 隧道反向等待 | 结构关联、纵向桥接耗尽/重捕获、零速确认及原始扫描；不要仅放宽完成条件 |
| 建图导航可用但保存INVALID | 实际mapping初始化时序、PCD/栅格数量和frame、reload seed，不要手改manifest状态 |
| 无显示或LiDAR异常 | headless仍需渲染；核对包内Ogre库/哈希、模型可视网格、自遮挡及40Hz批次时序 |
| 提示domain被占用 | 检查对应PID、命令和partition；只停止本次拥有的实例，不绕过文件锁 |
| 退出有SIGTERM/SIGKILL升级 | 保留日志并检查残留进程、最终零命令；不要把任务PASS写成优雅退出PASS |

本仓库禁止未经明确授权构建，也禁止Agent暂存、提交、推送、合并或覆盖已有工作区修改。本README只提供现有overlay的使用与只读调试方式。
