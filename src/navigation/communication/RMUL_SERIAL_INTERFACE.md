# RMUL 3V3 导航与 STM32 串口接口 V1

适用 2027 高校联盟赛 3V3。STM32 接收、校验并解析官方裁判包，把导航关心的状态打包成一个定长快照。导航发送底盘速度和升降目标。两类帧都是队内串口格式。

| 方向 | 帧头 | 整帧长度 | 内容 |
|---|---:|---:|---|
| 导航 → STM32 | `0x6E` | 16 字节 | 底盘系目标速度和云台升降目标 |
| STM32 → 导航 | `0x6D` | 21 字节 | 比赛阶段、自车状态、场地状态、RFID、控制度和云台升降到位反馈 |

使用 115200 baud、8N1，小端。两种帧沿用 Dust CRC16，初值 `0xFFFF`、反射多项式 `0x8408`、无最终异或。CRC 覆盖帧头到 CRC 字段前一字节，小端放于帧尾。

## C++ 整帧结构

```cpp
#include <cstddef>
#include <cstdint>

// 导航 → STM32
struct CommandPacket
{
  static constexpr std::size_t kSize = 16;
  static constexpr std::uint8_t kHeader = 0x6E;

  std::uint8_t header = kHeader;
  float linear_x = 0.0F;  // 底盘系 m/s，向前为正
  float linear_y = 0.0F;  // 底盘系 m/s，向左为正
  float angular_z = 0.0F; // rad/s，逆时针为正
  std::uint8_t lifter_target_down = 0; // 0 请求抬升，1 请求下降
  std::uint16_t checksum = 0;
} __attribute__((packed));

// STM32 → 导航
struct StatusPacket
{
  static constexpr std::size_t kSize = 21;
  static constexpr std::uint8_t kHeader = 0x6D;

  std::uint8_t header = kHeader;                          // 帧头固定 0x6D
  std::uint8_t sequence = 0;                              // 聚合帧序号，255 后回绕到 0
  std::uint8_t game_type = 0;                             // 0x0001 低四位，RMUL 3V3 为 4
  std::uint16_t stage_remain_s = 0;                       // 0x0001 当前阶段剩余秒数
  std::uint8_t game_progress = 0;                         // 0x0001 高四位，比赛中为 4
  std::uint16_t current_hp = 0;                           // 0x0201 当前血量，原始值
  std::uint16_t maximum_hp = 0;                           // 0x0201 最大血量，原始值
  std::uint8_t robot_id = 0;                              // 0x0201 本车 ID
  std::uint8_t chassis_power_enabled = 0;                 // 0x0201 底盘电源输出位，0 或 1
  std::uint16_t zone_state_and_rfid = 0;                  // 低 12 位为控制区状态，高 4 位为本车 RFID
  std::uint16_t ally_control_degree = 0;                  // 0x0003 己方控制度
  std::uint16_t enemy_control_degree = 0;                 // 0x0003 对方控制度
  std::uint8_t lifter_position = 0;                 // 0 抬升到位，1 下降到位，2 升降途中
  std::uint16_t checksum = 0;                            // 帧尾 CRC16，覆盖字节 0–18
} __attribute__((packed));

static_assert(sizeof(CommandPacket) == CommandPacket::kSize);
static_assert(sizeof(StatusPacket) == StatusPacket::kSize);
static_assert(offsetof(CommandPacket, checksum) == 14);
static_assert(offsetof(StatusPacket, checksum) == 19);
```

## 下发 0x6E

| 字节偏移 | 类型 | 数据 |
|---:|---|---|
| 0 | `uint8_t` | 固定 `0x6E` |
| 1 | `float` | `linear_x`，m/s，车头方向为正 |
| 5 | `float` | `linear_y`，m/s，车身左侧为正 |
| 9 | `float` | `angular_z`，rad/s，逆时针为正 |
| 13 | `uint8_t` | `lifter_target_down`，0 请求抬升，1 请求下降 |
| 14 | `uint16_t` | CRC16，覆盖字节 0–13 |

比赛通信使用带升降目标的 `0x6E`，完整帧见[下发结构体](include/utils/rmul_command_packet.hpp)。现有固定朝向联调程序仍按 [基础导航协议](BASIC_NAV_SERIAL_PROTOCOL.md) 发送 15 字节的 `0x6B`。

## 回传 0x6D

建议 STM32 以 20 Hz 发送聚合快照，分别接收各来源并汇总最近收到的数据。[完整帧结构体](include/utils/rmul_status_packet.hpp)

| 字节偏移 | 类型 | 字段 | 数据来源或语义 |
|---:|---|---|---|
| 0 | `uint8_t` | `header` | 固定 `0x6D` |
| 1 | `uint8_t` | `sequence` | 每发送一帧递增，255 后回到 0，用于观察丢帧，STM32 重启从 0 开始 |
| 2 | `uint8_t` | `game_type` | `0x0001` 低四位，RMUL 3V3 为 4 |
| 3 | `uint16_t` | `stage_remain_s` | `0x0001` 当前阶段剩余秒数 |
| 5 | `uint8_t` | `game_progress` | `0x0001` 高四位，比赛中为 4 |
| 6、8 | 各 `uint16_t` | `current_hp`、`maximum_hp` | `0x0201` 当前与最大血量，原始值 |
| 10 | `uint8_t` | `robot_id` | `0x0201` 当前机器人 ID |
| 11 | `uint8_t` | `chassis_power_enabled` | `0x0201` 底盘电源输出位，0 或 1 |
| 12 | `uint16_t` | `zone_state_and_rfid` | 见下方位定义 |
| 14、16 | 各 `uint16_t` | `ally_control_degree`、`enemy_control_degree` | `0x0003` 偏移 20、22，己方与对方控制度 |
| 18 | `uint8_t` | `lifter_position` | STM32 本车机构反馈，0 抬升到位，1 下降到位，2 升降途中 |
| 19 | `uint16_t` | `checksum` | CRC16，覆盖字节 0–18 |

### 过洞升降握手

1. 导航进入过洞准备后，下发 `lifter_target_down=1`。该值表示目标位置，保持到车辆确认驶离低净空区域。
2. STM32 根据机构的实际位置反馈生成 `lifter_position`。确认抬升到位回 `0`，确认下降到位回 `1`。正在抬升或下降时均回 `2`。按电控上电默认抬升的约定，回传初值为 `0`。STM32 拒绝 0 和 1 以外的升降目标。
3. 导航在收到有效且未超时的回传后，以 `lifter_position==1` 作为机械就绪条件。进入隧道还要满足路径与过洞准备条件，已有航向对齐要求时仍需另外满足。反馈超时或取值不是 `1` 时，过洞的下降到位条件不满足。
4. 确认驶离低净空区域后，导航下发 `lifter_target_down=0` 请求抬升。收到有效的 `lifter_position=0` 时确认抬升到位。

两个升降字段均来自本车导航和电控，属于队内协议。官方裁判系统的 RFID 数据仍用于场地检测。

### 场地与 RFID 位段

| 位 | 语义 | 官方来源 |
|---|---|---|
| 0–1、2–3 | 红方高地激活状态、控制状态 | `0x0101` 第二个 32 位字 bit 16–19 |
| 4–5、6–7 | 蓝方高地激活状态、控制状态 | 同上 bit 20–23 |
| 8–9、10–11 | 中央控制区激活状态、控制状态 | 同上 bit 24–27 |
| 12 | 本车检测到己方补给区卡 | `0x0209` 首个 32 位字 bit 0 |
| 13、14、15 | 本车检测到红方高地、蓝方高地、中央控制区卡 | 同上 bit 11、12、13 |

激活状态沿用官方编码，0 未激活、1 己方、2 对方、3 双方。控制状态为 0 中立、1 己方、2 对方、3 保留。红方、蓝方表示地图上的区域，己方、对方表示控制者。

### 接收校验

上位机按帧头、21 字节长度和 CRC 验证聚合帧。字段代表 STM32 缓存的最新值，聚合帧到达频率只反映队内串口链路。序号按聚合帧递增，和裁判系统官方包的序号无关。

定位参考刚体相对底盘的航向角仍按底盘定位接口单独定义。上位机已接入 `0x6E` 下发、`0x6D` 解析和串口速度门控。RMUL 决策侧已可通过状态订阅模式写入黑板快照，行为树消费待接入。启动和剩余工作见下文。

### 控制度与场地区域字段

`ally_control_degree` 是己方控制度，`enemy_control_degree` 是对方控制度。依据联盟赛规则 §3.3.4.2，双方开局各 600 点。每控制一处区域，每秒扣对方 1 点；一台机器人战亡，其所属队伍扣 10 点。STM32 转发官方给出的数值，上位机可计算双方差值。

`zone_state_and_rfid` 是一个 `uint16_t`。低 12 位编码三个区域的全场裁判状态，每区 2 位激活状态和 2 位控制状态。高 4 位是本车 RFID 检测结果。例如 bit 15 为 1 表示本车检测到了中央区卡，而中央区仍可处于中立状态。中央控制状态看 bit 10–11。

## 新增比赛数据在官方文件中的位置

以下页码以《RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（20261029）》页脚印刷页为准，括号内为 PDF 阅读器页序（含封面，从 1 开始）。官包偏移从各自**数据段**起算。

| 聚合帧中的新增数据 | 官方报文与位置 | 官包数据段定位 |
|---|---|---|
| 比赛类型、阶段、阶段剩余时间 | `0x0001`，表 1-6，印刷第 7–8 页（PDF 第 8–9 页） | 字节 0 的 bit 0–3 为类型、bit 4–7 为阶段；偏移 1 的 `uint16_t` 为剩余秒数 |
| 机器人 ID、当前血量、最大血量、底盘供电 | `0x0201`，表 1-12，印刷第 13 页（PDF 第 14 页） | 偏移 0、2、4；偏移 16 的 bit 1 为底盘电源输出 |
| 红方高地、蓝方高地、中央控制区的激活与控制状态 | `0x0101`，表 1-9，印刷第 11 页（PDF 第 12 页） | 偏移 4 的第二个 `uint32_t`，依次取 bit 16–19、20–23、24–27 |
| 本车补给区和三处控制区的 RFID 检测 | `0x0209`，表 1-19，印刷第 17–18 页（PDF 第 18–19 页） | 偏移 0 的第一个 `uint32_t`，取 bit 0、11、12、13 |
| 己方和对方控制度 | `0x0003`，表 1-8，印刷第 9 页（PDF 第 10 页） | 偏移 20、22，各为 `uint16_t` |

联盟赛规则手册 V1.0.0（20261015）§3.1.3（印刷第 20–21 页）规定哨兵参赛条件；§3.3.2（第 34–35 页）描述回血与复活；§3.3.4（第 36–37 页）描述三处控制区和控制度。帧头、序号、CRC 字段位置均属于队内串口定义。

## 编码测试向量

| 条件 | 线缆字节 |
|---|---|
| 下发零速并请求下降 | `6E 00 00 00 00 00 00 00 00 00 00 00 00 01 C2 40` |
| 回传确认下降到位，其余状态字段均为零 | `6D 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 20 E3` |
| 回传升降途中，其余状态字段均为零 | `6D 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 BB D1` |

## 实车接入与验收

Point-LIO 已运行、工作区构建并载入后，在实车导航入口显式选择 RMUL V1。

```bash
NAV_SERIAL=rmul_v1 NAV_SERIAL_PORT=/dev/ttyACM0 bash scripts/real_robot.bash navigation
ros2 launch bt_manager bt_manager.launch.py rmul_status_only:=true
ros2 topic echo /sentry/rmul_status
```

实车脚本同时启动定位、点云过滤、Nav2 和 RMUL 串口节点。另开终端运行决策订阅入口，比赛树和控制输出保持停用。升降目标使用已有的 `/sentry/behaivor_send`，字段为 `desire_lifter_pos`。默认升降目标为上位，收到新的合法目标后才更新。串口断流归零会保留进程内最后的升降目标。同一串口只启动一套节点。实车脚本不自动启动旧比赛行为树。

- 速度下发沿用固定零航向联调模式，角速度字段固定为零。开始前将车头对准 `camera_init` 的 X 正轴，并保持该朝向。
- 非零速度要求最近有效的 `0x6D` 在 250 ms 内、比赛类型为 4、阶段为 4、自身有有效非零血量、底盘供电开启。MPC 指令超时用原有 100 ms。250 ms 是本次按建议回传 20 Hz 选择的联调阈值，需按实车数据确认。
- 目标为上位时仅回传 0 才放行；目标为下位时要求 250 ms 内有有效行为指令且回传 1。若行为消息标出过洞准备，还要求该消息的 `tunnel_ready` 为真。反馈 2 时保持零速。
- 接收端验证 CRC，发布 `ros_interfaces/msg/RmulStatus` 到 `/sentry/rmul_status`。裁判字段分组的停更仍由 STM32 负责识别。建议 STM32 在关键裁判输入失效时停止发有效快照，车端速度看门狗也需独立验收。

下一步是让 RMUL 行为树消费状态快照、更新过洞位置反馈并给出可验证的航向对齐；再接入动态底盘航向和联盟赛自身的目标选择。若 STM32 或导航进程在低净空内重启，保持机构低位需要电控侧另行验证。当前接口的上电默认目标为抬升。
