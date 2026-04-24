// COBS (Consistent Overhead Byte Stuffing).
// Implementación canónica single-pass, idéntica a la de Jetson/Host.
//
// API:
//   cobs_encode(src, src_len, dst, dst_cap) -> bytes escritos, o 0 en overflow
//   cobs_decode(src, src_len, dst, dst_cap) -> bytes escritos, o 0 en error
//
// NO incluyen el delimitador 0x00 final. El caller lo añade al final del
// buffer encoded cuando se manda por serial.

#pragma once
#include <stdint.h>
#include <stddef.h>

namespace Protocol {

// Tamaño máximo del buffer encoded dado un payload de N bytes.
// Overhead COBS: +1 byte cada 254 bytes de entrada, más 1 byte fijo.
constexpr size_t cobs_max_encoded(size_t src_len) {
    return src_len + (src_len / 254) + 1;
}

// Retorna número de bytes escritos en dst, o 0 si dst_cap es insuficiente.
size_t cobs_encode(const uint8_t* src, size_t src_len,
                   uint8_t* dst, size_t dst_cap);

// Retorna número de bytes escritos en dst, o 0 si formato inválido
// (cero embebido, código pasa del final, etc.)
size_t cobs_decode(const uint8_t* src, size_t src_len,
                   uint8_t* dst, size_t dst_cap);

}  // namespace Protocol