#include "SerFrame.h"
#include "Cobs.h"
#include "Crc16.h"

#include <string.h>

namespace Protocol {

constexpr uint8_t DELIMITER = 0x00;

// Empaquetado: raw = [type][len][payload][crc16], COBS(raw), append 0x00.
size_t pack_frame(uint8_t type, const uint8_t* payload, size_t payload_len,
                  uint8_t* out, size_t out_cap) {
    if (payload_len > Cfg::MAX_FRAME_PAYLOAD) return 0;

    // raw temporal en stack (pequeño, bounded)
    uint8_t raw[2 + Cfg::MAX_FRAME_PAYLOAD + 2];
    const size_t raw_len = 2 + payload_len + 2;

    raw[0] = type;
    raw[1] = static_cast<uint8_t>(payload_len);
    if (payload_len) {
        memcpy(raw + 2, payload, payload_len);
    }
    const uint16_t crc = crc16_ccitt(raw, 2 + payload_len);
    // little-endian
    raw[2 + payload_len]     = static_cast<uint8_t>(crc & 0xFF);
    raw[2 + payload_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    // Necesitamos que out_cap contenga encoded + delimitador
    if (out_cap < cobs_max_encoded(raw_len) + 1) return 0;

    const size_t enc_len = cobs_encode(raw, raw_len, out, out_cap - 1);
    if (enc_len == 0) return 0;
    out[enc_len] = DELIMITER;
    return enc_len + 1;
}

// Stream parser
bool StreamParser::feed(uint8_t b) {
    if (b == DELIMITER) {
        bool ok = false;
        if (encLen_ > 0 && !overflow_) {
            ok = decodeCurrent();
            if (!ok) dropped_++;
        } else if (overflow_) {
            dropped_++;
        }
        encLen_ = 0;
        overflow_ = false;
        return ok;
    }
    if (overflow_) {
        // Seguimos consumiendo hasta el próximo delimitador
        return false;
    }
    if (encLen_ >= sizeof(encBuf_)) {
        overflow_ = true;
        return false;
    }
    encBuf_[encLen_++] = b;
    return false;
}

bool StreamParser::decodeCurrent() {
    // Buffer para raw post-COBS
    uint8_t raw[2 + Cfg::MAX_FRAME_PAYLOAD + 2 + 8];
    const size_t raw_len = cobs_decode(encBuf_, encLen_, raw, sizeof(raw));
    if (raw_len < 4) return false;  // mínimo: type+len+crc (payload vacío)

    const uint8_t type = raw[0];
    const uint8_t declared_len = raw[1];
    if (raw_len != static_cast<size_t>(2 + declared_len + 2)) return false;
    if (declared_len > Cfg::MAX_FRAME_PAYLOAD) return false;

    const uint16_t crc_lo = raw[2 + declared_len];
    const uint16_t crc_hi = raw[2 + declared_len + 1];
    const uint16_t crc_recv = static_cast<uint16_t>(crc_lo | (crc_hi << 8));
    const uint16_t crc_calc = crc16_ccitt(raw, 2 + declared_len);
    if (crc_recv != crc_calc) return false;

    type_ = type;
    payloadLen_ = declared_len;
    if (declared_len) {
        memcpy(payload_, raw + 2, declared_len);
    }
    return true;
}

}  // namespace Protocol