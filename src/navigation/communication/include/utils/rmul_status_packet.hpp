#pragma once

#include <cstddef>
#include <cstdint>

namespace ns_com::rmul
{
// STM32 -> navigation. Multibyte wire fields are little-endian.
struct StatusPacket
{
  static constexpr std::size_t kSize = 21;
  static constexpr std::uint8_t kHeader = 0x6D;

  std::uint8_t header = kHeader;
  std::uint8_t sequence = 0;
  std::uint8_t game_type = 0;
  std::uint16_t stage_remain_s = 0;
  std::uint8_t game_progress = 0;
  std::uint16_t current_hp = 0;
  std::uint16_t maximum_hp = 0;
  std::uint8_t robot_id = 0;
  std::uint8_t chassis_power_enabled = 0;
  std::uint16_t zone_state_and_rfid = 0;
  std::uint16_t ally_control_degree = 0;
  std::uint16_t enemy_control_degree = 0;
  std::uint8_t lifter_position = 0; // 0 抬升到位，1 下降到位，2 升降途中
  std::uint16_t checksum = 0;
} __attribute__((packed));

static_assert(sizeof(StatusPacket) == StatusPacket::kSize);
static_assert(offsetof(StatusPacket, zone_state_and_rfid) == 12);
static_assert(offsetof(StatusPacket, lifter_position) == 18);
static_assert(offsetof(StatusPacket, checksum) == 19);
}  // namespace ns_com::rmul
