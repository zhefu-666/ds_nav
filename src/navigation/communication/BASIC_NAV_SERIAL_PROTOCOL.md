# 基础导航串口协议 V1

用于上位机定位、规划和跟踪，STM32 执行底盘速度闭环。按 Dust 的定长帧习惯定义两种报文。定位和地图变换由上位机提供。

## 导航电脑与 STM32 的接口

| 方向 | 帧头 | 固定长度 | 数据 |
|---|---|---:|---|
| 导航 → STM32 | `0x6B` | 15 字节 | 底盘系目标速度 `linear_x`、`linear_y`、`angular_z` |
| STM32 → 导航 | `0x6C` | 7 字节 | `nav_frame_yaw_in_base_rad`，定位参考刚体相对底盘的航向角。定位参考刚体会相对底盘转动，且此角度由 STM32 测得时启用。 |

串口采用 115200、8N1。所有多字节字段小端，浮点为 IEEE 754 binary32。两个帧头是 V1 的新取值，可区分 Dust 原有的 `0x6A` 命令帧。

```cpp
#include <stddef.h>
#include <stdint.h>

// 导航电脑 -> STM32
struct CmdVelPacket
{
  static constexpr size_t kSize = 15;
  static constexpr uint8_t kHeader = 0x6B;

  uint8_t header = kHeader;     // 字节 0
  float linear_x = 0.0f;      // 字节 1..4，底盘系 m/s，前为正
  float linear_y = 0.0f;      // 字节 5..8，底盘系 m/s，左为正
  float angular_z = 0.0f;     // 字节 9..12，rad/s，逆时针为正
  uint16_t checksum = 0;       // 字节 13..14，覆盖字节 0..12
} __attribute__((packed));

// STM32 -> 导航电脑。启用串口航向反馈时周期发送。
struct ChassisYawPacket
{
  static constexpr size_t kSize = 7;
  static constexpr uint8_t kHeader = 0x6C;

  uint8_t header = kHeader;                 // 字节 0
  float nav_frame_yaw_in_base_rad = 0.0f;  // 字节 1..4，定位参考刚体相对底盘的 yaw
  uint16_t checksum = 0;                   // 字节 5..6，覆盖字节 0..4
} __attribute__((packed));

static_assert(sizeof(CmdVelPacket) == CmdVelPacket::kSize);
static_assert(sizeof(ChassisYawPacket) == ChassisYawPacket::kSize);
```

## 航向与坐标转换

速度指令统一参考底盘 `base_link`，X 向前、Y 向左、Z 向上。上位机先取得底盘在速度参考坐标系中的航向，再把平移目标旋转到底盘系。

- 定位参考刚体和底盘刚性固定，或上位机已有可信的动态 TF 时，上位机直接使用该变换。此时基础导航只需要下发速度帧。
- 定位参考刚体会相对底盘转动，且 STM32 掌握相对航向时，回传 `0x6C`。字段表示定位参考刚体的前向轴相对底盘前向轴的角度，绕 +Z 逆时针为正，单位 rad，范围 `[-π, π)`。编码器零点和安装偏角在生成该字段时处理。它是两刚体间的**相对角**。接入前需确认 STM32 有相应的相对角测量来源。
- 在平面航向模型下，底盘航向等于定位参考刚体的航向减去回传的相对角。例：参考坐标系 X 向东，车头朝北，目标向东 1 m/s，则下发 `linear_x=0`、`linear_y=-1`。
- 世界系速度和 Point-LIO 的位姿、线速度、角速度留在上位机，供定位与 MPC 使用。下位机的编码器或电机估计速度留在本地速度环。

## 校验和失效处理

- CRC 沿用本机 Dust 实现的参数，初值 `0xFFFF`、反射多项式 `0x8408`、结果无异或。覆盖从帧头到 CRC 前一个字节，CRC 低字节先发。
- 收端按帧头选择固定长度，校验 CRC 和浮点有限性。CRC 失败时从候选帧的下一字节重新找帧头。
- 停车时三个目标速度都发零。STM32 仅用有效速度帧刷新指令看门狗，超时后将目标速度置零。启用串口航向反馈时，上位机在相对角无效或过期时发送零速，直到取得可用航向。
- 建议联调起点为指令 50 Hz、启用时航向反馈 50 Hz、两端超时 100 ms。最终周期和超时由实车测量确认。

## 测试向量

| 输入 | 线缆字节 |
|---|---|
| 零速 | `6B 00 00 00 00 00 00 00 00 00 00 00 00 7A C5` |
| `vx=1.0`、`vy=-0.5`、`wz=0.25` | `6B 00 00 80 3F 00 00 00 BF 00 00 80 3E 40 99` |
| 相对航向 `0.5 rad` | `6C 00 00 00 3F 7F 2F` |

[当前运行协议](SERIAL_PROTOCOL.md)中的既有报文由原收发代码处理。V1 上位机已接入固定零航向联调模式，始终发送 `angular_z=0`，这一阶段只使用 `0x6B`。启动与验收见[交付文档](BASIC_NAV_SERIAL_DELIVERY.md)。
