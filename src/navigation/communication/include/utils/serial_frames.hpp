#pragma once

#include "protocol.hpp"

// 导航电脑视角的串口整帧布局。载荷字段见 custom_protocol.hpp。
// 帧头 8 字节，data_len 只计载荷。多字节字段按小端传输。
// checksum 位于包头偏移 6，取包头前 6 字节和载荷的 16 位累加和。
namespace ns_com::serial_frames
{
#pragma pack(push, 1)

// 上位机 -> STM32
struct SendNavigationPacket
{
  PacketHeader header;        // A5 5A 07 00 00 2D + checksum
  ChassisTarget payload;      // 45 字节，目标速度、里程计和航向差
};

struct SendBehaviorPacket
{
  PacketHeader header;        // A5 5A 07 00 08 19 + checksum
  BehaviorData payload;       // 25 字节，行为决策
};

// STM32 -> 上位机。类型值对应当前接收回调的分支。
struct ReceiveAllyStatusPacket
{
  PacketHeader header;        // 0x01
  TeamInfo payload;
};

struct ReceiveGameStatusPacket
{
  PacketHeader header;        // 0x02
  GameInfo payload;
};

struct ReceiveSentryOnlinePacket
{
  PacketHeader header;        // 0x03
  SentryInfoOnline payload;
};

struct ReceiveSentrySelfPacket
{
  PacketHeader header;        // 0x04
  SentryInfoOffline payload;
};

struct ReceiveRadarPacket
{
  PacketHeader header;        // 0x05
  RadarInfo payload;
};

#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 8);
static_assert(sizeof(SendNavigationPacket) == 53);
static_assert(sizeof(SendBehaviorPacket) == 33);
static_assert(sizeof(ReceiveAllyStatusPacket) == 56);
static_assert(sizeof(ReceiveGameStatusPacket) == 30);
static_assert(sizeof(ReceiveSentryOnlinePacket) == 45);
static_assert(sizeof(ReceiveSentrySelfPacket) == 38);
static_assert(sizeof(ReceiveRadarPacket) == 67);
}  // namespace ns_com::serial_frames
