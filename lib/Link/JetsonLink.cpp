#include "JetsonLink.h"

#include <string.h>

void JetsonLink::begin(uint32_t baud) {
    // Buffers mayores al default (64 bytes) para absorber ráfagas.
    Serial2.setRxBufferSize(Cfg::SERIAL_RX_BUFFER);
    Serial2.setTxBufferSize(Cfg::SERIAL_TX_BUFFER);
    Serial2.begin(baud);

    // Esperamos un poco a que Serial2 esté listo (algunos cores lo requieren).
    // No bloqueamos indefinidamente por si está corriendo headless.
    const uint32_t t0 = millis();
    while (!Serial2 && (millis() - t0) < 500) {
        delay(1);
    }
}

void JetsonLink::tick() {
    // Drenamos todo lo disponible de una vez. Como el parser es por byte,
    // lo bucleamos. Limitamos iteraciones por tick para no monopolizar el
    // loop() si llega una ráfaga enorme.
    size_t budget = 256;
    while (Serial2.available() > 0 && budget-- > 0) {
        const int b = Serial2.read();
        if (b < 0) break;
        if (parser_.feed(static_cast<uint8_t>(b))) {
            lastRxMs_ = millis();
            framesRx_++;
            dispatchFrame();
        }
    }
}

void JetsonLink::dispatchFrame() {
    const uint8_t t = parser_.type();
    const uint8_t* p = parser_.payload();
    const size_t n = parser_.payloadLen();

    switch (t) {
        case Cfg::MsgType::MOTOR_CMD: {
            if (n < 6) return;  // esperamos 3 int16 little-endian
            // Cuidado con strict aliasing: lo hacemos byte-a-byte.
            int16_t L = static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
            int16_t R = static_cast<int16_t>(static_cast<uint16_t>(p[2]) | (static_cast<uint16_t>(p[3]) << 8));
            int16_t A = static_cast<int16_t>(static_cast<uint16_t>(p[4]) | (static_cast<uint16_t>(p[5]) << 8));
            if (cbMotor_) cbMotor_(L, R, A, ctxMotor_);
            break;
        }
        case Cfg::MsgType::BRAKE_ON:
            if (cbBrake_) cbBrake_(ctxBrake_);
            break;
        case Cfg::MsgType::HEARTBEAT:
            if (cbHb_) cbHb_(ctxHb_);
            break;
        default:
            // Tipos desconocidos se ignoran. No corrompen el stream porque el
            // framing COBS+CRC ya validó el frame.
            break;
    }
}

bool JetsonLink::sendTelemetry(const uint8_t* payload, size_t len) {
    return sendRaw(Cfg::MsgType::TELEMETRY, payload, len);
}

bool JetsonLink::sendHello() {
    return sendRaw(Cfg::MsgType::ESP_HELLO, nullptr, 0);
}

bool JetsonLink::sendRaw(uint8_t type, const uint8_t* payload, size_t len) {
    // Buffer de salida dimensionado para el peor caso COBS + delimitador.
    // pack_frame escribe: cobs_encoded(raw) + 1 byte delimitador.
    // raw_len máximo = 2 + MAX_PAYLOAD + 2 = 244.
    // cobs_max = raw_len + ceil(raw_len/254) + 1 -> con margen 260 sobra.
    static constexpr size_t OUT_CAP = Cfg::MAX_FRAME_PAYLOAD + 16;
    uint8_t buf[OUT_CAP];
    const size_t n = Protocol::pack_frame(type, payload, len, buf, sizeof(buf));
    if (n == 0) return false;
    // Serial2.write devuelve los bytes escritos; si el TX buffer está lleno
    // bloquea hasta poder, pero en práctica a 921600 baud casi nunca pasa.
    return Serial2.write(buf, n) == n;
}