#pragma once

#include "dust_serial_crc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace ns_com::basic_nav
{
struct CmdVelPacket
{
  static constexpr std::size_t kSize = 15;
  static constexpr std::uint8_t kHeader = 0x6B;

  std::uint8_t header = kHeader;
  float linear_x = 0.0F;    // 底盘系 m/s，向前为正
  float linear_y = 0.0F;    // 底盘系 m/s，向左为正
  float angular_z = 0.0F;   // rad/s，固定朝向联调模式下始终为零
  std::uint16_t checksum = 0;
} __attribute__((packed));

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(CmdVelPacket) == CmdVelPacket::kSize);
static_assert(offsetof(CmdVelPacket, checksum) == 13);

using Frame = std::array<std::uint8_t, CmdVelPacket::kSize>;

inline void encodeFloat(float value, std::uint8_t * output)
{
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t i = 0; i < sizeof(bits); ++i) {
    output[i] = static_cast<std::uint8_t>(bits >> (8 * i));
  }
}

inline Frame encodeFixedHeadingCommand(float linear_x, float linear_y)
{
  Frame frame{};
  frame[0] = CmdVelPacket::kHeader;
  encodeFloat(linear_x, frame.data() + offsetof(CmdVelPacket, linear_x));
  encodeFloat(linear_y, frame.data() + offsetof(CmdVelPacket, linear_y));

  constexpr auto checksum_offset = offsetof(CmdVelPacket, checksum);
  const auto crc = dustSerialCrc(frame.data(), checksum_offset);
  frame[checksum_offset] = static_cast<std::uint8_t>(crc);
  frame[checksum_offset + 1] = static_cast<std::uint8_t>(crc >> 8);
  return frame;
}
}  // namespace ns_com::basic_nav
