#pragma once

#include <cstddef>
#include <cstdint>

namespace ns_com
{
inline std::uint16_t dustSerialCrc(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<std::uint16_t>((crc >> 1) ^ ((crc & 1) ? 0x8408 : 0));
    }
  }
  return crc;
}
}  // namespace ns_com
