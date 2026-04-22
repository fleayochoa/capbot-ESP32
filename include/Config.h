// Constantes de configuración del firmware.
//
// Todos los valores en un solo sitio, sin #defines mágicos dispersos.

#pragma once
#include <stdint.h>

namespace Cfg {

// -------- Serial con Jetson --------
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr size_t   SERIAL_RX_BUFFER = 1024;   // ESP32 Serial por defecto es pequeño
constexpr size_t   SERIAL_TX_BUFFER = 1024;

// -------- Watchdog del link con Jetson (requisito ME ms) --------
// Si no llega nada (MOTOR_CMD ni HEARTBEAT) en este tiempo, FRENO ACTIVO.
constexpr uint32_t JETSON_WATCHDOG_MS = 200;

// -------- Telemetría --------
// Frecuencia con la que armamos y mandamos el paquete TELEMETRY.
// 50 Hz = periodo 20 ms. Coincide con el requisito de publicación.
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;

// -------- PWM motores --------
constexpr uint32_t PWM_FREQ_HZ = 20000;   // 20 kHz → fuera del audible
constexpr uint8_t  PWM_RESOLUTION_BITS = 10;  // 0..1023

// Rango del comando de motor (int16 del host → PWM interno).
// El host manda valores en [-32000, 32000]; los mapeamos a [-1023, 1023].
constexpr int32_t CMD_FULL_SCALE = 32000;

// -------- Tamaños de buffer del framing --------
constexpr size_t MAX_FRAME_PAYLOAD = 240;  // TELEMETRY JSON cabe holgado
constexpr size_t RX_BUFFER_BYTES = 512;

// -------- Tipos de mensaje serial (mantener sincronizado con Jetson) --------
namespace MsgType {
    constexpr uint8_t MOTOR_CMD  = 0x10;  // Jetson -> ESP32
    constexpr uint8_t BRAKE_ON   = 0x11;  // Jetson -> ESP32
    constexpr uint8_t HEARTBEAT  = 0x12;  // Jetson -> ESP32
    constexpr uint8_t TELEMETRY  = 0x20;  // ESP32 -> Jetson
    constexpr uint8_t ESP_HELLO  = 0x21;  // ESP32 -> Jetson
}

}  // namespace Cfg