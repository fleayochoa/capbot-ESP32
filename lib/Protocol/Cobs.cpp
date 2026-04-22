#include "Cobs.h"

namespace Protocol {

size_t cobs_encode(const uint8_t* src, size_t src_len,
                   uint8_t* dst, size_t dst_cap) {
    if (dst_cap == 0) return 0;

    size_t code_idx = 0;   // índice del byte "code" del bloque actual
    size_t out_idx = 1;    // cursor de escritura (skip code byte)
    uint8_t code = 1;      // cuenta code + bytes no-cero ya incluidos

    // placeholder del primer code
    dst[0] = 0;

    for (size_t i = 0; i < src_len; ++i) {
        const uint8_t b = src[i];
        if (b == 0) {
            // Cierra bloque actual
            dst[code_idx] = code;
            if (out_idx >= dst_cap) return 0;
            code_idx = out_idx;
            dst[out_idx++] = 0;  // placeholder para el siguiente code
            code = 1;
        } else {
            if (out_idx >= dst_cap) return 0;
            dst[out_idx++] = b;
            code++;
            if (code == 0xFF) {
                // Bloque lleno (254 bytes no-cero): cierra con marcador 0xFF
                // que le dice al decoder "NO reinsertes cero al abrir el
                // siguiente bloque".
                dst[code_idx] = code;
                if (out_idx >= dst_cap) return 0;
                code_idx = out_idx;
                dst[out_idx++] = 0;
                code = 1;
            }
        }
    }
    dst[code_idx] = code;
    return out_idx;
}

size_t cobs_decode(const uint8_t* src, size_t src_len,
                   uint8_t* dst, size_t dst_cap) {
    size_t in = 0;
    size_t out = 0;
    while (in < src_len) {
        const uint8_t code = src[in];
        if (code == 0) {
            return 0;  // cero embebido = stream inválido
        }
        const size_t end = in + code;
        if (end > src_len) {
            return 0;  // code pasa del final
        }
        // Copiar bytes del bloque (sin incluir el code)
        for (size_t j = in + 1; j < end; ++j) {
            if (out >= dst_cap) return 0;
            dst[out++] = src[j];
        }
        in = end;
        // Reinserta cero entre bloques salvo que (a) bloque == 0xFF
        // (marcador de bloque completo sin cero) o (b) ya estamos al final.
        if (code < 0xFF && in < src_len) {
            if (out >= dst_cap) return 0;
            dst[out++] = 0;
        }
    }
    return out;
}

}  // namespace Protocol