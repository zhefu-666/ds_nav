#pragma once

#include <cstddef>
#include <cstdint>

namespace ns_com::rmul
{
// Navigation -> STM32. Multibyte wire fields are little-endian.
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

static_assert(sizeof(float) == 4);
static_assert(sizeof(CommandPacket) == CommandPacket::kSize);
static_assert(offsetof(CommandPacket, lifter_target_down) == 13);
static_assert(offsetof(CommandPacket, checksum) == 14);
}  // namespace ns_com::rmul
