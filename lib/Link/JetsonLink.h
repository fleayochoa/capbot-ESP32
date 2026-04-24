// Puente entre el Serial y el resto del firmware.
//
// Responsabilidades:
//   - Alimentar bytes del Serial al StreamParser
//   - Despachar callbacks por tipo de mensaje
//   - Mantener el timestamp del último RX válido (para el watchdog)
//   - Ofrecer API de envío de frames (telemetría, hello)
//
// El despacho se hace con punteros a función (no std::function) para evitar
// allocations en hot-path. Cada callback recibe un contexto opaco (void*)
// que fija el usuario al registrar.

#pragma once
#include <Arduino.h>
#include <stdint.h>

#include "Config.h"
#include "SerFrame.h"

class JetsonLink {
public:
    using MotorCallback     = void (*)(int16_t left, int16_t right, int16_t aux, void* ctx);
    using VoidCallback      = void (*)(void* ctx);

    JetsonLink() = default;

    // Llamar en setup(). Configura Serial y buffers.
    void begin(uint32_t baud = Cfg::SERIAL_BAUD);

    // Llamar en cada iteración de loop(). Lee lo disponible del Serial,
    // lo alimenta al parser y dispara callbacks cuando completa un frame.
    void tick();

    // ---- Registro de callbacks ----
    void onMotorCmd(MotorCallback cb, void* ctx)   { cbMotor_  = cb; ctxMotor_  = ctx; }
    void onBrake   (VoidCallback  cb, void* ctx)   { cbBrake_  = cb; ctxBrake_  = ctx; }
    void onHeartbeat(VoidCallback cb, void* ctx)   { cbHb_     = cb; ctxHb_     = ctx; }

    // ---- Envío ----
    // Manda TELEMETRY con el payload dado. Devuelve true si se envió.
    bool sendTelemetry(const uint8_t* payload, size_t len);

    // Manda ESP_HELLO. Se llama una vez en setup() para avisar a la Jetson.
    bool sendHello();

    // ---- Estado ----
    // millis() del último frame válido recibido. 0 si nada aún.
    uint32_t lastRxMs() const { return lastRxMs_; }

    // Diagnóstico
    uint32_t framesRx() const { return framesRx_; }
    uint32_t framesDropped() const { return parser_.droppedFrames(); }

private:
    bool sendRaw(uint8_t type, const uint8_t* payload, size_t len);
    void dispatchFrame();

    Protocol::StreamParser parser_;

    MotorCallback cbMotor_ = nullptr;  void* ctxMotor_ = nullptr;
    VoidCallback  cbBrake_ = nullptr;  void* ctxBrake_ = nullptr;
    VoidCallback  cbHb_    = nullptr;  void* ctxHb_    = nullptr;

    uint32_t lastRxMs_ = 0;
    uint32_t framesRx_ = 0;
};