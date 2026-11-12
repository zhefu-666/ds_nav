# ds_nav 实车接入与远程调试操作手册

适用：Ubuntu 22.04、ROS 2 Humble、双 Mid-360、Point-LIO、GICP、Nav2＋MINCO＋MPC。更新：2026-11-12。

本手册按当前源码编写。部署衔接经过静态和模拟命令检查；新机器编译、插件加载、传感器、定位、实车导航与避障仍需现场验证。命令中的示例地址不是已探测到的机器人地址。

## 1. 先认清电脑、网络与入口

- **机器人电脑**：连接雷达、底盘；运行驱动、LIO、地图、定位、导航、通信。以下未特别标注的命令均在这里执行。
- **开发电脑**：SSH登录机器人；可运行RViz。SSH通了不代表DDS发现和数据传输通了。
- **雷达网口**：建议独立连接交换机和两台雷达；管理网络使用另一网口或Wi-Fi。不要通过正在承载SSH的网口直接改地址。
- `scripts/real_robot.bash mapping`：需要先启动LIO；启动二维建图和地图保存，不启动自动探索、底盘通信或比赛决策。
- `scripts/real_robot.bash navigation`：需要先启动LIO；启动GICP、过滤、先验导航，不启动底盘通信或比赛决策。
- `start.bash`：桌面比赛入口，会启动PTP、传感器、定位、导航、决策、通信；首次接入不要直接用它代替分模块检查。

请先记录：机器人SSH地址/用户、工作区绝对路径、雷达网口名及MAC、主机雷达网IP、两台雷达序列号/IP/前后角色、底盘串口、固件协议、ROS_DOMAIN_ID、地图版本与原点、实际外参。地址改动前保存原值。

## 2. 获取代码和准备环境

### 2.1 Clone与版本确认

开发电脑：

```bash
ssh <机器人用户>@<机器人管理IP>
```

机器人电脑：

```bash
git clone https://github.com/zhefu-666/ds_nav.git ~/ds_nav
cd ~/ds_nav
git log -1 --format='%H %s'
git status --short
```

须使用包含本手册及部署修复的版本。只clone旧提交470f0cd7不会得到后续工作区修改。`build/`、`install/`不随Git分发。实车不依赖`src/simulation`子模块；需要完整仿真才单独初始化子模块。

### 2.2 安装依赖

先按[README安装章节](README.md#️-安装与构建)准备ROS 2 Humble、编译器、系统库。下列命令会安装系统软件，需在目标机器人上执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y tmux linuxptp ethtool iputils-arping tcpdump \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  libeigen3-dev libpcl-dev libyaml-cpp-dev libgoogle-glog-dev \
  libunwind-dev libfmt-dev libopencv-dev libomp-dev \
  ros-humble-backward-ros ros-humble-behaviortree-cpp-v3 \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-spatio-temporal-voxel-layer \
  ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox
# 首次初始化才执行；已初始化时跳过这一行
sudo rosdep init
rosdep update
rosdep install --from-paths src/navigation src/perception src/decision \
  src/ros_interfaces src/utils/pcd2pgm --ignore-src -r -y
```

还需要C++版[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)、[small_gicp](https://github.com/koide3/small_gicp)、[Sophus](https://github.com/strasdat/Sophus)。按各项目安装说明安装，记录版本；同名Python包不能代替C++头文件/库。不要从另一架构电脑复制`.so`。README有补充依赖清单；rosdep完成不能证明全部外部库存在。

### 2.3 构建与安装验证

以下是真正触发编译的操作；Agent执行前须用户明确许可，本手册撰写时没有执行：

```bash
cd ~/ds_nav
BUILD_SCOPE=real bash build.bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix navi2
ros2 pkg prefix point_lio
ros2 pkg prefix icp_relocalization
ros2 pkg prefix minco_planner
ros2 pkg prefix minco_controller
ros2 pkg prefix communication
ros2 pkg prefix bt_manager
ls "$(ros2 pkg prefix --share navi2)/behavior_tree"
ls "$(ros2 pkg prefix --share navi2)/scripts/savemap.bash"
ls "$(ros2 pkg prefix --share icp_relocalization)/pcd"
```

`BUILD_SCOPE=real`包含实车依赖和pcd2pgm；不构建仿真集成包。默认不设置该变量会构建完整工作区。报缺库先修复对应依赖，不要删依赖或禁用检查。构建成功也不等于插件可加载或实车可运动。

## 3. 雷达地址如何更换

### 3.1 当前仓库默认值及对应位置

| 项目 | 当前值 | 改哪里 |
|---|---|---|
| 机器人雷达网IP | 192.168.1.47 | JSON的`MID360.host_net_info`五个`*_ip` |
| 前雷达 | 192.168.1.135 | JSON的`lidar_configs[].ip`、驱动YAML的`merge_front_ip` |
| 后雷达 | 192.168.1.122 | JSON的`lidar_configs[].ip`、驱动YAML的`merge_back_ip` |
| 主IMU | livox/imu_192_168_1_135 | 通用LIO launch的`pointlio_imu_topic`参数 |
| PTP网口 | enp86s0 | `PTP_INTERFACE`环境变量 |

JSON：[mixed_MID360_config.json](src/perception/livox_ros_driver2/config/mixed_MID360_config.json)。驱动YAML：[mixed_MID360_component.yaml](src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml)。LIO参数：[mid360.yaml](src/perception/Point-LIO/config/mid360.yaml)。

**修改JSON不等于修改雷达设备自身IP。** 建议先保留雷达现有地址，仅调整主机与驱动配置；只有现场网络要求时才改设备地址。

### 3.2 找到真实网口和雷达

```bash
ip -br link
ip -br addr
ip route
nmcli device status
# 下例enp3s0必须替换为真实雷达网口
ethtool enp3s0
ethtool -T enp3s0
ip neigh show dev enp3s0
```

看：链路是否UP、协商速率、现有网段、网卡是否支持硬件时间戳/PHC。`ip neigh`为空不等于无设备；可能尚未发生ARP通信。已知旧地址时：

```bash
sudo arping -I enp3s0 -c 3 192.168.1.135
sudo arping -I enp3s0 -c 3 192.168.1.122
sudo timeout 5s tcpdump -ni enp3s0 -c 20 'arp or udp'
```

看MAC回应及UDP源地址。ping失败不能单独判雷达离线；最终以驱动收到新鲜点云/IMU为准。未知IP时用Livox Viewer 2设备发现、设备序列号和抓包交叉确认，不猜历史地址。

### 3.3 配置机器人网口静态地址

以下统一采用**示例**：主机192.168.25.100/24、前雷达192.168.25.136、后雷达192.168.25.137，网口enp3s0。三者不能冲突。若保持原网段，则替换成原网段的有效值。

NetworkManager管理的Ubuntu可新建专用配置；先确认该网口不承载当前SSH：

```bash
nmcli connection show
sudo nmcli connection add type ethernet ifname enp3s0 \
  con-name ds-nav-lidar ipv4.method manual \
  ipv4.addresses 192.168.25.100/24 ipv4.never-default yes \
  ipv6.method disabled
sudo nmcli connection up ds-nav-lidar
ip -br addr show enp3s0
ip route get 192.168.25.136
```

已有专用连接时用`nmcli connection modify <连接名> ...`修改，不反复创建同名连接。雷达专网不设置默认网关/DNS，保留管理网络路由。非NetworkManager系统按实际netplan/networkd管理方式配置，不同时让两套服务管理网卡。

### 3.4 修改雷达设备自身地址（确实需要时）

1. 从[Livox官方下载页](https://www.livoxtech.com/mid-360/downloads)取得Viewer 2及Mid-360手册；按手册接线与供电。
2. 停止占用该雷达的驱动/Viewer；一次处理一台，按序列号确认前后雷达。
3. 电脑先进入雷达旧IP所在子网，例如旧192.168.1.*网段；打开Viewer 2并确认能看到该设备。
4. 选中目标设备，进入设备设置，设置静态IP；前雷达填示例192.168.25.136，后雷达填192.168.25.137。具体控件名称以所用Viewer版本为准。
5. 按工具提示应用/重连；若提示重启，按提示重启。不要同时修改无法区分的两台设备。
6. 电脑切换到新子网192.168.25.100/24，重新发现设备，核对序列号、新地址、点云。另一台重复操作。
7. 更新下面的驱动配置；Viewer验证后关闭Viewer再启动ROS驱动。

官方操作依据：[Mid-360用户手册v1.2](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)连接与IP设置章节。接线、电源和静态IP修改以官方手册为准。

### 3.5 创建自己的配置，避免直接改默认比赛参数

```bash
mkdir -p ~/ds_nav_config ~/ds_nav_maps
cp src/perception/livox_ros_driver2/config/mixed_MID360_config.json ~/ds_nav_config/lidar.json
cp src/perception/livox_ros_driver2/config/mixed_MID360_component.yaml ~/ds_nav_config/driver.yaml
cp src/perception/Point-LIO/config/mid360.yaml ~/ds_nav_config/lio.yaml
cp src/perception/icp_relocalization/config/gicp_relocalization.yaml ~/ds_nav_config/gicp.yaml
cp src/navigation/navi2_bringup/params/sentry1.yaml ~/ds_nav_config/nav.yaml
cp src/perception/msg_convert/config/cloud_registered_crop_filter.yaml ~/ds_nav_config/filter.yaml
```

在`~/ds_nav_config/lidar.json`中修改：

```json
"host_net_info": {
  "cmd_data_ip": "192.168.25.100", "cmd_data_port": 56100,
  "push_msg_ip": "192.168.25.100", "push_msg_port": 56200,
  "point_data_ip": "192.168.25.100", "point_data_port": 56300,
  "imu_data_ip": "192.168.25.100", "imu_data_port": 56400,
  "log_data_ip": "192.168.25.100", "log_data_port": 56500
}
```

这是原JSON中`MID360.host_net_info`的替换片段，不是完整文件。`lidar_configs`中原`.122`改为后雷达`.137`，原`.135`改为前雷达`.136`，保留其余字段。UDP端口保持原配置；不要只修改其中一个host IP字段。

在`driver.yaml`的`/**.ros__parameters`下修改：

```yaml
user_config_path: /home/实际用户/ds_nav_config/lidar.json
merge_front_ip: 192.168.25.136
merge_back_ip: 192.168.25.137
```

路径必须是真实绝对路径；YAML中`~`不会自动像shell那样展开。保留`merge_output_topic: livox/lidar`、`xfer_format: 1`及现有接口。前后雷达换角色会改变外参方向，不能只对调IP。

主IMU话题随雷达IP改变，示例前雷达话题为`/livox/imu_192_168_25_136`。用下面的**通用launch**显式覆盖：

```bash
ros2 launch point_lio livox_pointlio_intra_process.launch.py \
  driver_params_file:="$HOME/ds_nav_config/driver.yaml" \
  pointlio_params_file:="$HOME/ds_nav_config/lio.yaml" \
  pointlio_imu_topic:=livox/imu_192_168_25_136
```

不要用`mixed_livox_pointlio_intra_process.launch.py`包装入口覆盖这三个参数：它内部固定传入默认配置和旧IMU话题。`lio.yaml`中的`common.imu_topic`也应更新便于独立启动一致，但通用launch传入的参数优先。

检查配置语法：

```bash
python3 -m json.tool ~/ds_nav_config/lidar.json >/dev/null
python3 - <<'PY'
from pathlib import Path
import yaml
for p in (Path.home()/'ds_nav_config').glob('*.yaml'):
    yaml.safe_load(p.read_text())
    print('OK', p)
PY
```

仅改外部配置不需重新编译；改C++外参/串口则需重新构建相应包并重新source。

## 4. 外参与时间同步

### 4.1 必须同步核对的几何配置

| 配置 | 作用 | 检查方式 |
|---|---|---|
| driver.merge_extrinsic_back_to_front | 后雷达到前雷达 | 测量/标定，墙面叠合，确认方向和数组单位 |
| lio.mapping.extrinsic_T/R | LIO雷达/IMU外参 | 按真实传感器定义，不能用底盘偏置替代 |
| lio.preprocess.blind_center | 近距离盲区中心 | 与真实底盘/雷达安装匹配 |
| Point-LIO laserMapping.cpp的offset_vec | camera_init到base_link计算中的底盘偏置 | 当前[0,0.20,0]，变更需源码构建 |
| nav.yaml中planner/controller杆臂 | 雷达与控制中心关系 | 对照README几何说明，不能只改一处 |
| ROGMap Z窗口/中心 | 占据、投影和ESDF高度覆盖 | 在RViz看地面、障碍、坡面与窗口是否合理 |

数组单位/顺序查[驱动实现](src/perception/livox_ros_driver2)与[双雷达标定说明](src/perception/dual_lidar_calibration/README.md)，不要把SDK JSON外参和融合YAML外参重复施加。详细耦合见[README](README.md)的上车前关键配置。

### 4.2 PTP

先用`ethtool -T enp3s0`确认网卡硬件能力；不支持时不能把启动成功当硬件同步通过。独立终端：

```bash
sudo env PTP_INTERFACE=enp3s0 bash scripts/ptp_sync.bash
```

该脚本会关闭NTP/chrony/timesyncd并重启已有ptp4l/phc2sys，运行前确认机器没有其他依赖这些服务的任务。另一个终端看：

```bash
tail -n 40 /tmp/ptp4l_enp3s0.log
tail -n 40 /tmp/phc2sys_enp3s0.log
```

查链路/PHC错误、端口状态、offset是否持续稳定。双雷达时间戳类型、点内时间与主IMU新鲜度还需独立核对；PTP进程存在不证明全部同步。结束后按原系统配置恢复时间服务，不能不加区分地同时启用多个时间守护进程。

## 5. 每个终端的ROS环境

```bash
cd ~/ds_nav
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42   # 示例值，同一实车系统保持一致，避开其他机器人
export ROS_LOCALHOST_ONLY=0  # 需要跨电脑DDS时；仅机器人本机调试可按现场策略设1
printf '%s\n' "$AMENT_PREFIX_PATH"
printenv RMW_IMPLEMENTATION
ros2 node list
```

不用仿真`/clock`，节点`use_sim_time`应为false。Domain42不是项目强制值；不要照搬旧仿真域87或本机MCP隔离域188。明确选择RMW后，各端保持可兼容配置；不要误source其他工作区。长任务使用tmux。原有服务正在运行时先确认归属，不再启动第二套。

## 6. 逐段检查传感器与LIO

启动第3.5节的通用launch后，在独立终端检查：

```bash
ros2 component list
ros2 node list
ros2 topic list -t
ros2 topic info /livox/lidar -v
ros2 topic info /livox/imu_192_168_25_136 -v
timeout 5s ros2 topic hz /livox/lidar
timeout 5s ros2 topic hz /livox/imu_192_168_25_136
ros2 topic echo /livox/imu_192_168_25_136 --once --field header
ros2 topic info /aft_mapped_to_init -v
timeout 5s ros2 topic hz /aft_mapped_to_init
ros2 topic echo /aft_mapped_to_init --once
ros2 topic info /cloud_registered_full -v
ros2 topic echo /cloud_registered_full --once --field header
```

`echo --once`可能一直等数据，无输出时Ctrl+C；可在命令前加`timeout 5s`。`timeout`到期退出码124是有界采样结束，不自动表示链路故障；要看期间是否收到数据。

需要看到：

- 两台雷达确实被识别；`/livox/lidar`是融合CustomMsg，主IMU来自选定前雷达。
- stamp持续推进，无明显回跳/停滞；时间域与系统及TF一致。点内时间问题还要看驱动/LIO日志和专门诊断。
- `/aft_mapped_to_init`持续更新，静置不明显漂移/翻转；配置100Hz不代表实测一定100Hz。
- `/cloud_registered_full`在`camera_init`，墙面与地面位置合理。`/cloud_registered`是另一条降采样链，不能混当完整点云。
- `camera_init -> base_link`、`camera_init -> body`、`camera_init -> slambase`可查询；传感器容器名为`livox_pointlio_container`，导航planner会装入此容器。

```bash
timeout 5s ros2 run tf2_ros tf2_echo camera_init base_link
timeout 5s ros2 run tf2_ros tf2_echo camera_init slambase
```

先修传感器/LIO，再开定位和导航；不要用增大优化容差掩盖上游点云或时间错误。

## 7. 建图和保存

### 7.1 二维地图

确保没有GICP、map_server、另一套slam_toolbox在运行；LIO保持运行：

```bash
MAP_OUTPUT_DIR="$HOME/ds_nav_maps" USE_RVIZ=false bash scripts/real_robot.bash mapping
tmux attach -t ds_nav_real
```

有机器人显示环境时可`USE_RVIZ=true`。该模式只建图，车辆移动由现场明确授权的人工控制完成，不发送Nav2任务。链路：

```text
双Mid-360 → Livox融合 → Point-LIO /cloud_registered_full
    → pointcloud_to_laserscan /scan → slam_toolbox /map
    → savemap.bash → ~/ds_nav_maps/slam_<时间>.yaml + .pgm
```

```bash
ros2 topic info /scan -v
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once --field header
ros2 topic info /map -v
ros2 topic echo /map --once --field info
timeout 5s ros2 run tf2_ros tf2_echo map camera_init
```

看`/scan` frame为slambase、有效量程内有墙面；地图宽高/分辨率/原点有效且逐渐扩展，闭环后无明显重影。默认每60秒保存；也可手动保存：

```bash
bash scripts/savemap.bash /map "$HOME/ds_nav_maps/site_01"
ls -lh ~/ds_nav_maps/site_01.*
```

查看YAML的`image`确实指向已存在的PGM，origin/resolution与地图一致。地图文件生成不代表地图质量通过。

### 7.2 三维PCD供GICP使用

只保存二维PGM不能直接给GICP。采集前在自己的`lio.yaml`中找到已有的`pcd_save`分组，设置`pcd_save_en: true`；保留正确节点层级，按采集时长选择已有`interval`参数。`interval: -1`会累计到结束保存，长时间采集可能占用大量内存。

重启LIO加载该配置会重置局部坐标，必须重新开始同一轮建图。当前实现把分段PCD写到编译期Point-LIO源码根下`PCD/scans_<编号>.pcd`，正常退出时把尚未保存部分写到`PCD/scans.pcd`；后者不保证包含此前所有分段。使用Ctrl+C正常退出后核对日志和文件，不能只取最后残余文件当完整地图。保留完整分段、对应二维地图及坐标变换记录。

GICP目标PCD与二维地图必须处于一致的map坐标约定。slam_toolbox闭环可改变二维轨迹，而原始LIO点云不会自动得到相同全局修正；二者不能仅靠文件同名认定对齐。需完成地图配准/处理，实测固定参照点验证后，再设置GICP initial_pose。

## 8. 从新地图进入先验导航

### 8.1 完成配置配套

编辑自己的`nav.yaml`：

- `planner_server.ros__parameters.MincoPlanner`下ROGMap投影先验`prior_map.yaml_path`改为新先验二维地图绝对路径，PGM通过该YAML解析；查看原文件实际嵌套，不随意增加重复键。
- 保留PRIORMAP导航模式；核对地图范围、ROGMap中心/Z窗口、机器人半径和杆臂。
- Nav2的`NAV_MAP`是另一项，必须同时指向所需静态地图；两份地图可有不同用途，但必须同坐标系。

编辑`gicp.yaml`：

- `target_pcd_file`：经过处理并验证坐标的PCD绝对路径。
- `initial_pose`：当前上电位置在目标地图中的[x,y,z,roll,pitch,yaw]，角度按配置使用弧度；实测填写。
- 保留定位质量/稳定性/初始位姿保护门槛，检查质量分布，不能为了得到TF而放宽门槛。

编辑`filter.yaml`：现有`positions`、`box_sizes`是旧赛场的裁剪区域，`remove_inside: true`会删除框内点。新场地必须逐个核对/重设，并在RViz观察markers和过滤前后障碍。不要未经确认直接复制旧框或把数组清空当作可靠禁用（实现有单框回退）。

`navigation2.launch.py`中`map -> minimap`仍是蓝方赛场固定值`[28.7,15,0,3.14,0,0]`，与比赛坐标/决策相关。新场地必须核对；修改该源码后需更新安装包，不要拿它替代`map -> camera_init`。未完成比赛坐标适配时不要启动比赛决策。

### 8.2 退出建图，再启动定位导航

在ds_nav_real每个窗口Ctrl+C，确认相关进程退出；再关闭旧会话：

```bash
ros2 node list
# 确认窗口内launch均已结束后才执行；不要靠断SSH停车
tmux kill-session -t ds_nav_real
NAV_MAP="$HOME/ds_nav_maps/site_01.yaml" \
NAV_PARAMS="$HOME/ds_nav_config/nav.yaml" \
GICP_CONFIG="$HOME/ds_nav_config/gicp.yaml" \
FILTER_CONFIG="$HOME/ds_nav_config/filter.yaml" \
USE_RVIZ=false bash scripts/real_robot.bash navigation
```

LIO需仍在独立终端运行；如果为保存PCD已停止LIO，先重新启动并按实际上电位置更新GICP初值。GICP与slam_toolbox不可同时发布`map -> camera_init`。

脚本启动进程后应逐个窗口查看日志，不会自动判定位就绪。节点名检查只是减少误启动，不保证全DDS网络TF发布者唯一。

## 9. 导航、避障与底盘链路怎么看

```text
目标：RViz/Nav2 action 或 bt_manager
  → Nav2 BT → ComputePathToPose → MINCO planner
地图：/cloud_registered_full → ROGMap occupancy/projection/ESDF → 搜索与轨迹优化
  → /opt_path → MPC + /aft_mapped_to_init → /cmd_vel_mpc
  → communication（另行启动）→ 串口 → 下位机
辅助costmap：/cloud_registered → 全局STVL
             /cloud_registered → crop filter → /cloud_registered_filtered → 局部STVL
定位：PCD + /cloud_registered_full → GICP → map到camera_init
```

### 9.1 定位和TF先过关

```bash
ros2 node info /gicp_relocalization_node
ros2 param get /gicp_relocalization_node target_pcd_file
ros2 param get /gicp_relocalization_node initial_pose
timeout 5s ros2 run tf2_ros tf2_echo map camera_init
timeout 5s ros2 run tf2_ros tf2_echo map base_link
```

查看GICP日志中的收敛、inlier/overlap、稳定窗口与初值保护拒绝原因，并查看`~/.ros/gicp_relocalization_results.csv`。有TF不代表定位可信；静置时与真实地图位置一致且稳定后再看规划。不要把超时、旧TF或默认位姿当作成功。

### 9.2 点云、地图、避障

```bash
ros2 topic info /cloud_registered -v
ros2 topic info /cloud_registered_filtered -v
ros2 node info /cloud_registered_crop_filter
ros2 topic list | rg 'rog_map|costmap|opt_path|mpc'
ros2 topic info /rog_map/occupied -v
ros2 topic info /opt_path -v
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

在RViz同时看原始/过滤点云、crop markers、占据/ESDF可用显示、全局/局部costmap、轨迹。静态物体应出现在正确位置，移动物体离开后地图应按配置清除，过滤不能删除测试障碍。先用静置机器人和人工摆放障碍检查输入，再进行获准的受控移动避障。

局部STVL无输入不等于ROGMap也没输入；ROGMap正常也不能掩盖局部costmap缺失。沿上图逐段查发布者、订阅者、QoS及时间，不要只看topic名字存在。

### 9.3 任务与控制

```bash
ros2 action list -t
ros2 action info /navigate_to_pose
ros2 topic echo /opt_path --once --field header
ros2 topic echo /cmd_vel_mpc --once
ros2 topic info /cmd_vel_mpc -v
```

实际发送目标会使后续通信可能驱动车辆，必须完成现场运动授权、急停和定位检查。优先用RViz的Nav2目标工具，先单个低速短距离任务；取消用RViz Cancel。核对任务状态、到点误差、轨迹新鲜度、取消后速度以及实际底盘停止。

`/opt_path`是自定义轨迹消息，不是普通Path；可视化看`/opt_path_vis`、`/astar_path_vis`、`/mpc_predict_path`。`/cmd_vel_mpc`保留世界系速度语义，communication会结合方向信息转换，不能在未知坐标约定下直接比较电控轴速度。

### 9.4 底盘接入与比赛决策

```bash
ls -l /dev/serial/by-id/ /dev/ttyACM* 2>/dev/null
id
```

当前串口在[com.cpp](src/navigation/communication/src/com.cpp)固定为`/dev/ttyACM0`，115200。先核对设备/权限及下位机固件协议；如需更改串口，修改该处并重新构建communication，不存在现成串口launch参数。串口权限按现场用户组配置，例如加入dialout后需重新登录；不要长期用sudo运行整个ROS系统。

在轮组架空或明确受控场地并确认独立急停后，才启动：

```bash
ros2 launch communication com.launch.py
```

看收发状态、下位机速度反馈和超时停车；串口能打开不证明协议或速度转换正确。基础到点/取消/停车/避障通过后，再按[bt_manager说明](src/decision/bt_manager/README.md)核对实际XML、场地坐标、姿态与裁判协议，启动比赛决策：

```bash
ros2 launch bt_manager bt_manager.launch.py
```

停止/取消、控制消息断流、传感器/定位异常和SSH断线时的底盘处理必须独立验证，不能用仿真停止结果替代。

## 10. 远程可视化和录包

开发电脑安装兼容ROS及Nav2 RViz插件；DDS网络可达、Domain一致后运行：

```bash
source /opt/ros/humble/setup.bash
source ~/ds_nav/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
rviz2 -d "$(ros2 pkg prefix --share navi2)/config/our.rviz"
```

建图用`config/slam.rviz`。导航Fixed Frame设map；只查LIO时先设camera_init。RobotModel需要真实URDF/robot_state_publisher；没有车模时先禁用该显示，不影响点云/地图显示。不应启用与Point-LIO重复的动态TF。

DDS不通时先在机器人本机看topic和录包，检查路由、防火墙及实际RMW发现配置；不要把SSH TCP转发当DDS隧道。可选Foxglove需安装并运行Bridge后使用SSH转发，但它不自动解决RViz订阅。

在机器人本地录包，按存储情况裁减点云：

```bash
mkdir -p ~/ds_nav_bags
ros2 bag record -o "$HOME/ds_nav_bags/robot_$(date +%Y%m%d_%H%M%S)" \
  /tf /tf_static /aft_mapped_to_init /cloud_registered_full \
  /opt_path /opt_path_vis /cmd_vel_mpc /map
```

按实际topic追加主IMU和裁判状态。Ctrl+C后`ros2 bag info <目录>`检查时长、消息数，再复制到开发电脑。在隔离ROS域回放；不要向实车控制域回放速度/行为消息。不要将大体积bag及现场配置密钥提交Git。

## 11. 常见现象与排查顺序

| 现象 | 先看 | 不应先做 |
|---|---|---|
| 雷达无数据 | 网口UP、主机IP五字段、设备真实IP、路由/ARP、UDP与端口占用 | 先调LIO滤波参数 |
| 点云有但LIO不动 | IMU实际话题与launch覆盖、时间戳、主IMU来源、日志 | 只改YAML却忽略launch覆盖 |
| 双墙/漂移 | 融合外参方向、时间同步、LIO外参和静置稳定 | 改地图概率掩盖 |
| GICP不收敛 | PCD内容与坐标、初始位姿、输入新鲜度和质量指标 | 放宽收敛准入 |
| costmap无障碍 | 对应topic发布者、过滤框、frame/QoS、Z窗口 | 直接增大避障代价 |
| 有地图无路径 | lifecycle状态、BT XML安装、地图原点、起终点可通行性、ESDF | 重启全部节点掩盖日志 |
| 有轨迹无车动 | MPC新鲜度、communication是否启动、串口协议与下位机状态 | 直接注入更大速度 |
| RViz空白 | 本机/机器人各自topic、Domain、DDS、Fixed Frame、TF、插件 | 认定导航失败 |
| 找不到behavior_tree/保存脚本 | 安装规则是否为修复后版本、是否重新构建source | 依赖当前工作目录绕过 |

## 12. 应按什么顺序阅读源码/文档

1. [README](README.md)：安装、实车几何耦合、参数索引和已知限制。
2. [real_robot.bash](scripts/real_robot.bash)、[start.bash](start.bash)：SSH与桌面实际启用路径。
3. [通用LIO launch](src/perception/Point-LIO/launch/livox_pointlio_intra_process.launch.py)：配置优先级、容器、主IMU；再看[Point-LIO说明](src/perception/Point-LIO/README.md)。
4. [GICP launch](src/perception/icp_relocalization/launch/gicp_relocalization.launch.py)、[配置](src/perception/icp_relocalization/config/gicp_relocalization.yaml)、[说明](src/perception/icp_relocalization/README.md)：PCD、初值、质量门槛。
5. [navigation2.launch.py](src/navigation/navi2_bringup/launch/navigation2.launch.py) → [navigation_launch.py](src/navigation/navi2_bringup/launch/navigation_launch.py) → [sentry1.yaml](src/navigation/navi2_bringup/params/sentry1.yaml)：实际插件/资源/输入。
6. [MINCO说明](src/navigation/minco_planner/README.md)、[MPC说明](src/navigation/minco_controller/README.md)、[ROGMap说明](src/perception/rog_map/README.md)：规划、控制和地图语义。
7. [通信说明](src/navigation/communication/README.md)、[决策说明](src/decision/bt_manager/README.md)：基础导航验证后再接比赛策略。
8. [slam.launch.py](src/navigation/navi2_bringup/launch/slam.launch.py)、[slam.yaml](src/navigation/navi2_bringup/params/slam.yaml)、[保存脚本](scripts/savemap.bash)：二维建图链。

同名历史配置不代表正在生效。遇到问题先从启动入口追到实际参数文件，再查源代码读写者。

## 13. 完成一轮现场接入应留下的结果

记录代码提交号和未提交diff、系统/SDK版本、配置副本与地图哈希、雷达序列号/地址/安装外参、实际topic/frame/频率、定位质量、任务结果、取消/停车和障碍测试结果、日志与bag路径。分别标明“通过、失败、未测”。

阶段顺序：依赖构建 → 静置传感器/LIO → 建图质量/保存 → 地图配准与定位 → 地图障碍输入 → 受控到点/取消 → 受控避障/故障停车 → 比赛策略。前一阶段失败先修该阶段，不为了得到PASS放宽门槛。

## 14. 本轮文档整理与变更记录

本手册合并替代本地实车速查和本轮远程部署改造流水记录；其他任务记录、架构文档、仿真验收材料未删除。README保留入口链接，避免操作步骤多处重复。

部署代码修复涉及资源安装、包路径解析、默认过滤输入、SSH入口、可选RViz、实车构建范围和PTP网卡覆盖；没有改比赛行为树内容、planner/controller算法、QoS、接口名和避障阈值。本手册同时保留本轮必要审计结论：Bash/launch/XML/YAML及diff检查通过，临时安装资源解析和模拟SSH窗口检查通过；没有执行编译和实车运行，不能宣称功能已现场验收。未执行构建：AGENTS.md禁止未授权构建。

文档位于仓库根目录，不被`docs/`忽略规则屏蔽；未执行git add/commit/push，须由仓库维护者自行审阅并提交，远端clone才能获得这些工作区修改。
