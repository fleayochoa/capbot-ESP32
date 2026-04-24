// CRC16-CCITT (poly 0x1021, init 0xFFFF).
// Mismo algoritmo que en Jetson/Host (Python).
// Vector de test: CRC("123456789") == 0x29B1

#pragma once
#include <stdint.h>
#include <stddef.h>

namespace Protocol {

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);

}  // namespace Protocol