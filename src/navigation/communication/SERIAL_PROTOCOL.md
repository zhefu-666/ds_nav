# 导航电脑与 STM32 串口对接协议

本文按当前导航电脑源码整理。发送和接收方向均以导航电脑为准。基础导航对接优先确认 **下发 `0x00`、回传 `0x04`**。运行通信节点时也会周期下发 `0x08`。启用比赛行为树时，按策略需要提供其他状态包。

## 基础导航最小对接

| 方向 | 现有完整帧 | 导航直接使用的数据 |
|---|---|---|
| 导航 → STM32 | `0x00`，53 字节 | 目标世界系平移速度 `vx_mps/vy_mps` 和目标角速度 `vw_rpm`。同帧携带 `current_yaw/delta_yaw`，用于表达里程计与底盘航向关系。 |
| STM32 → 导航 | `0x04`，38 字节 | `chassis_imu_yaw`。上位机用它与 Point-LIO 航向计算 `delta_yaw`。 |

`current_vx/current_vy/current_vw` 由 Point-LIO 提供，MPC 在上位机本地使用。当前 `0x00` 同时下发这三个状态字段；STM32 是否在控制环中使用它们，需要查固件。

启用过洞联动时，`0x08` 下发对齐请求与目标角，`0x04` 回传 `lifter_current_pos` 和 `tunnel_yaw_aligned`。其余状态帧供比赛行为树使用。通信循环仍周期发送 `0x08`。

下方参考 Dust 的 `packet.hpp`，每种报文用一个带 `__attribute__((packed))` 的结构体列清字段。沿用本项目的包头、包型及校验规则。

## 串口与通用包头

- 设备 `/dev/ttyACM0`，115200 baud，8N1。
- 帧为 8 字节包头加 `data_len` 字节载荷。整数和 32 位浮点字段按小端传输，`bool` 占 1 字节，字段之间无填充。下表的偏移以字节计。
- 校验为包头前 6 字节与载荷按小端 16 位字累加后取低 16 位。载荷长度为奇数时，最后一个字节按无符号数加入。发送端写入校验和，当前上位机接收端的 `check()` 直接返回成功。

| 帧偏移 | 类型 | 字段 |
|---:|---|---|
| 0–1 | `uint8_t[2]` | 固定 `A5 5A` |
| 2 | `uint8_t` | `from`，导航电脑发送时为 `0x07` |
| 3 | `uint8_t` | `to`，导航电脑发送时为 `0x00` |
| 4 | `uint8_t` | `packet_type` |
| 5 | `uint8_t` | `data_len`，仅载荷长度 |
| 6 | `uint16_t` | `checksum` |

## 导航电脑 → STM32

通信节点每 10 ms 依次提交导航控制帧和行为决策帧到串口写缓冲，标称 100 Hz。

```cpp
#include <stddef.h>
#include <stdint.h>

struct SendNavigationPacket
{
  static constexpr size_t kSize = 53;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x00;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x07;         // 导航电脑
  uint8_t to = 0x00;           // STM32
  uint8_t packet_type = kType;
  uint8_t data_len = 45;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  float vx_mps = 0.0f;                        // 世界系 X 方向目标速度，m/s
  float vy_mps = 0.0f;                        // 世界系 Y 方向目标速度，m/s
  float vw_rpm = 0.0f;                        // 目标旋转角速度，rpm
  float current_yaw = 0.0f;                   // 里程计当前航向，实际发送单位为度
  float current_vx = 0.0f;                    // 里程计 X 线速度，m/s
  float current_vy = 0.0f;                    // 里程计 Y 线速度，m/s
  float current_vw = 0.0f;                    // 里程计角速度，rad/s
  float fx_global = 0.0f;                     // /cmd_force_mpc 的 force.x
  float fy_global = 0.0f;                     // /cmd_force_mpc 的 force.y
  float fw_global = 0.0f;                     // /cmd_force_mpc 的 torque.z
  bool use_speed_control = true;              // 当前发送端固定为 true
  float delta_yaw = 0.0f;                     // 里程计航向减底盘 IMU 航向，度，范围 ±180°
} __attribute__((packed));
static_assert(sizeof(SendNavigationPacket) == SendNavigationPacket::kSize);

struct SendBehaviorPacket
{
  static constexpr size_t kSize = 33;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x08;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x07;         // 导航电脑
  uint8_t to = 0x00;           // STM32
  uint8_t packet_type = kType;
  uint8_t data_len = 25;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  uint8_t pitch_mode = 0;                     // 云台抬头模式
  uint8_t desire_stance = 0;                  // 哨兵姿态
  uint8_t desire_lifter_pos = 0;              // 云台升降状态
  float scan_yaw_min_deg = 0.0f;              // 扫描最小偏航角（度）
  float scan_yaw_max_deg = 0.0f;              // 扫描最大偏航角（度）
  uint16_t ammo_purchase_request = 0;         // 弹药兑换请求（递增累计值）
  uint8_t revive_request = 0;                 // 是否确认复活
  uint8_t remote_revive_request = 0;          // 远程复活请求
  uint8_t remote_ammo_request = 0;            // 远程买弹请求
  uint8_t remote_health_request = 0;          // 远程买血请求
  bool use_limited_scan = false;              // 是否使用限制性扫描模式
  bool not_aim_enemy = false;                 // 是否瞄准敌方目标
  bool use_capacitor = false;                 // 是否使用电容
  bool tunnel_align_active = false;           // 请求过洞航向对齐
  float tunnel_align_angle_deg = 0.0f;        // 过洞航向对齐目标角，度
} __attribute__((packed));
static_assert(sizeof(SendBehaviorPacket) == SendBehaviorPacket::kSize);
```

## STM32 → 导航电脑

`0x04` 用于底盘航向反馈，过洞还用同一帧的升降和对齐状态。`0x01`、`0x02`、`0x03`、`0x05` 分别用于队友、比赛、自身在线和雷达状态。回传结构中的收发方初值用双方的枚举 ID 表示方向，当前接收分发依据包类型。

```cpp
#include <stddef.h>
#include <stdint.h>

struct ReceiveSentrySelfPacket
{
  static constexpr size_t kSize = 38;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x04;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x00;         // STM32，接收端按帧内实际值读取
  uint8_t to = 0x07;           // 导航电脑，接收端按帧内实际值读取
  uint8_t packet_type = kType;
  uint8_t data_len = 30;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  bool is_get = false;                        // 视觉是否瞄准到敌人
  float armor_pos[3]{};                       // 瞄准到的装甲板位置，具体坐标系询问视觉
  uint8_t armor_num = 0;                      // 不做红蓝方区分
  float yaw_camerainit_to_gimbal = 0.0f;      // 编码器 yaw，逆时针为正
  uint8_t lifter_current_pos = 0;             // 0 -- kTop 1 -- kBottom 2 -- kMiddle
  bool is_transformable = false;              // 是否能够进行变形（不止变形中不能变形，升降卡住后也无法进行变形）
  float transform_state = 0.0f;               // 变形状态，0-1，0%为未变形，100%为完全变形，过渡状态根据实际情况变化
  uint8_t capacitor_capacity = 0;             // 电容容量百分比，0-100
  float chassis_imu_yaw = 0.0f;               // 基础导航所需。底盘 IMU yaw，度，逆时针为正
  bool tunnel_yaw_aligned = false;            // 过洞航向对齐完成反馈
} __attribute__((packed));
static_assert(sizeof(ReceiveSentrySelfPacket) == ReceiveSentrySelfPacket::kSize);

struct AllyRobotStatus
{
  uint8_t robot_id = 0;                       // 机器人ID，蓝方=红方+100
  uint16_t robot_hp = 0;                      // 机器人血量
  float robot_pos_x = 0.0f;                   // 机器人位置x坐标，单位m
  float robot_pos_y = 0.0f;                   // 机器人位置y坐标，单位m
} __attribute__((packed));

struct EnemyRobotStatus
{
  uint8_t robot_id = 0;                       // 机器人ID，蓝方=红方+100
  uint16_t robot_hp = 0;                      // 机器人血量
  uint16_t allowed_projectile = 0;            // 允许发射的弹丸数量，单位发
  uint16_t robot_pos_x = 0;                   // 机器人位置x坐标，单位cm
  uint16_t robot_pos_y = 0;                   // 机器人位置y坐标，单位cm
} __attribute__((packed));

struct ReceiveAllyStatusPacket
{
  static constexpr size_t kSize = 56;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x01;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x00;         // STM32，接收端按帧内实际值读取
  uint8_t to = 0x07;           // 导航电脑，接收端按帧内实际值读取
  uint8_t packet_type = kType;
  uint8_t data_len = 48;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  AllyRobotStatus ally_status[4]{};           // 4个友方机器人状态
  uint16_t outpost_hp = 0;                    // 前哨站血量
  uint16_t base_hp = 0;                       // 基地血量
} __attribute__((packed));
static_assert(sizeof(ReceiveAllyStatusPacket) == ReceiveAllyStatusPacket::kSize);

struct ReceiveGameStatusPacket
{
  static constexpr size_t kSize = 30;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x02;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x00;         // STM32，接收端按帧内实际值读取
  uint8_t to = 0x07;           // 导航电脑，接收端按帧内实际值读取
  uint8_t packet_type = kType;
  uint8_t data_len = 22;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  uint16_t game_time_remaining = 0;           // 比赛剩余时间，单位s
  uint16_t coin_remaining = 0;                // 己方剩余金币数量
  uint32_t event_code = 0;                    // 场地事件代码，未解码，需接收后根据协议解码
  uint8_t game_status = 0;                    // 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中
  float manual_point_x = 0.0f;                // 手动指定的目标点x坐标，单位m，坐标系minimap
  float manual_point_y = 0.0f;                // 手动指定的目标点y坐标，单位m，坐标系minimap
  uint8_t manual_key = 0;                     // 手动指定的目标点快捷键
  uint16_t enemy_outpost_hp = 0;              // 敌方前哨站血量
  uint16_t enemy_base_hp = 0;                 // 敌方基地血量
} __attribute__((packed));
static_assert(sizeof(ReceiveGameStatusPacket) == ReceiveGameStatusPacket::kSize);

struct ReceiveSentryOnlinePacket
{
  static constexpr size_t kSize = 45;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x03;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x00;         // STM32，接收端按帧内实际值读取
  uint8_t to = 0x07;           // 导航电脑，接收端按帧内实际值读取
  uint8_t packet_type = kType;
  uint8_t data_len = 37;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  uint16_t self_health = 0;                   // 机器人血量
  uint16_t bullets_remaining = 0;             // 剩余发弹量
  uint16_t cooling_value = 0;                 // 机器人每秒冷却值
  uint16_t heat_limit = 0;                    // 机器人热量上限
  uint16_t current_heat = 0;                  // 机器人当前热量
  float sentry_pos_x = 0.0f;                  // 哨兵位置x坐标，单位m
  float sentry_pos_y = 0.0f;                  // 哨兵位置y坐标，单位m
  float speed_monitor_angle = 0.0f;           // 测速模块朝向，单位为度，正北为0度
  uint32_t sentry_info_1 = 0;                 // 哨兵信息1，未解码，需接收后根据协议解码
  uint16_t sentry_info_2 = 0;                 // 哨兵信息2，未解码，需接收后根据协议解码
  uint64_t sentry_info_3 = 0;                 // 哨兵信息3，未解码，需接收后根据协议解码
  uint8_t energy_ratio = 0;                   // 底盘能量比例
} __attribute__((packed));
static_assert(sizeof(ReceiveSentryOnlinePacket) == ReceiveSentryOnlinePacket::kSize);

struct ReceiveRadarPacket
{
  static constexpr size_t kSize = 67;     // 线缆上的整帧字节数
  static constexpr uint16_t kHeader = 0x5AA5;  // 小端字节 A5 5A
  static constexpr uint8_t kType = 0x05;

  uint16_t start_of_frame = kHeader;
  uint8_t from = 0x00;         // STM32，接收端按帧内实际值读取
  uint8_t to = 0x07;           // 导航电脑，接收端按帧内实际值读取
  uint8_t packet_type = kType;
  uint8_t data_len = 59;        // 仅载荷字节数
  uint16_t checksum = 0;       // 16 位累加和，覆盖头部前 6 字节和载荷

  EnemyRobotStatus enemy_status[6]{};         // 英雄 工程 步兵3 步兵4 无人机 哨兵
  uint16_t enemy_coin_left = 0;               // 敌方剩余金币数量
  uint16_t enemy_coin_accumulated = 0;        // 敌方累计获得金币数量
  bool is_enemy_outpost_sensed = false;       // 敌方前哨站是否被雷达识别到
} __attribute__((packed));
static_assert(sizeof(ReceiveRadarPacket) == ReceiveRadarPacket::kSize);
```

基础导航也应发送完整 30 字节载荷。通信节点以收包时刻给 IMU 样本打戳，与 Point-LIO 的里程计时间戳做最近匹配，默认窗口 40 ms。首次匹配成功前，通信节点把 `vx/vy` 和 `fx/fy` 清零；后续匹配失败时沿用最后一次有效航向差。仓库上位机源码未规定 STM32 的回传频率，实际应结合车上时间戳匹配情况确认。

## 源码索引

- [整帧结构](include/utils/serial_frames.hpp)和[各包完整载荷定义](include/utils/custom_protocol.hpp)
- [通用包头及校验](include/utils/protocol.hpp)和[串口收发分支](src/com.cpp)
- [ROS 订阅、组包与航向匹配](include/com_interface_ros.hpp)
- [串口节点启动入口](launch/com.launch.py)

协议以当前工作区源码为依据。STM32 固件版本、串口实测频率和实车对接尚未核验。
