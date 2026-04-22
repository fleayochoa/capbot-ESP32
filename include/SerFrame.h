// Frame serial: pack/unpack de [type:1][len:1][payload:len][crc16:2]
// dentro de un bloque COBS + delimitador 0x00.
//
// Stream parser: alimentas bytes uno a uno con feed() y cuando devuelve true
// puedes leer el frame decodificado vía type(), payload(), payloadLen().

#pragma once
#include <stdint.h>
#include <stddef.h>

#include "Config.h"

namespace Protocol {

// --- Empaquetado ---
// out debe tener espacio para cobs_max_encoded(2 + payload_len + 2) + 1 byte
// (delimitador). Devuelve bytes escritos, o 0 en error.
size_t pack_frame(uint8_t type, const uint8_t* payload, size_t payload_len,
                  uint8_t* out, size_t out_cap);

// --- Stream parser incremental ---
class StreamParser {
public:
    StreamParser() = default;

    // Llamar con cada byte recibido. Devuelve true cuando se completó un
    // frame válido (y puede leerse con type()/payload()/payloadLen()).
    // Los frames corruptos se descartan silenciosamente.
    bool feed(uint8_t b);

    uint8_t type() const { return type_; }
    const uint8_t* payload() const { return payload_; }
    size_t payloadLen() const { return payloadLen_; }

    // Contadores útiles para diagnóstico
    uint32_t droppedFrames() const { return dropped_; }

private:
    // Buffer para bytes COBS-encoded (antes de decodificar)
    uint8_t encBuf_[Cfg::RX_BUFFER_BYTES];
    size_t  encLen_ = 0;
    bool    overflow_ = false;

    // Resultado del frame más reciente
    uint8_t type_ = 0;
    uint8_t payload_[Cfg::MAX_FRAME_PAYLOAD];
    size_t  payloadLen_ = 0;

    uint32_t dropped_ = 0;

    bool decodeCurrent();  // intenta decodificar lo acumulado, rellena type_/payload_
};

}  // namespace Protocol