#include "pico_protocol.h"

uint16_t pico_crc16_ccitt_false(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xffffu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (unsigned bit = 0; bit < 8u; ++bit) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}
