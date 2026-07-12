// Constantes de configuración del firmware.
//
// Todos los valores en un solo sitio, sin #defines dispersos.

#pragma once
#include <stdint.h>

namespace Cfg {

// -------- Serial con Jetson --------
constexpr uint32_t SERIAL_BAUD = 460800;
constexpr size_t   SERIAL_RX_BUFFER = 1024;   // ESP32 Serial por defecto es 64 bytes, insuficiente para nuestros frames JSON.
constexpr size_t   SERIAL_TX_BUFFER = 1024;

// -------- Watchdog del link con Jetson (requisito ME ms) --------
// Si no llega nada (MOTOR_CMD ni HEARTBEAT) en este tiempo, FRENO ACTIVO.
constexpr uint32_t JETSON_WATCHDOG_MS = 200;

// -------- Telemetría --------
// Frecuencia con la que armamos y mandamos el paquete TELEMETRY.
// 50 Hz = periodo 20 ms.
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;

// -------- PWM motores --------
constexpr uint32_t PWM_FREQ_HZ = 500;   // 500 Hz
constexpr uint8_t  PWM_RESOLUTION_BITS = 10;  // 0 - 1023

// Rango del comando de motor (int16 del host → PWM interno).
// El host manda valores en [-32768, 32767]; los mapeamos a [-1023, 1023].
constexpr int32_t CMD_FULL_SCALE = 32768;

// -------- Tamaños de buffer del framing --------
constexpr size_t MAX_FRAME_PAYLOAD = 384;  // TELEMETRY JSON con odo + setpoint + PID gains
constexpr size_t RX_BUFFER_BYTES = 512;

// -------- Control de velocidad nav2 (AUTONOMOUS_NAV) --------
// Tiempo máximo sin un VEL_CMD fresco en este modo antes de frenar. Cubre
// el caso en que nav2 deja de publicar /cmd_vel pero el link con la Jetson
// (heartbeat) sigue vivo, lo cual JETSON_WATCHDOG_MS no detecta.
// 1000 ms: el último setpoint queda bufferizado (g_wheelSp) y el PID lo
// sigue aplicando entre ciclos del planner (nav2 publica a ~5 Hz = 200 ms;
// con 300 ms cualquier jitter frenaba el robot a tirones). Es solo un
// fail-safe: el puente de la Jetson ya manda un VEL_CMD(0,0) explícito a
// los 500 ms si /cmd_vel se corta.
constexpr uint32_t NAV_VEL_TIMEOUT_MS = 1000;

// -------- Cinemática de ruedas --------
// Cuentas por revolución con decodificación 4x (QuadratureEncoder cuenta los
// 4 flancos A/B por ciclo). Usado para convertir cuentas/seg del encoder a
// rad/s de rueda en el PID de velocidad on-board y en la odometría.
constexpr float WHEEL_CPR = 898.0f;

// Diámetro de rueda (m). Legacy: radio ~0.0335 m (ver ganancias del PID).
constexpr float WHEEL_DIAMETER_M = 0.067f;

// Separación entre ruedas / track width (m): distancia entre los puntos de
// contacto de la rueda izquierda y derecha. Usado por la odometría on-board
// para derivar la velocidad de giro (yaw) a partir de los encoders
// (omega = (vR - vL) / WHEEL_BASE_M). Medido en hardware.
constexpr float WHEEL_BASE_M = 0.17625f;

// -------- Odometría (encoders + IMU) --------
// Se calcula on-board y se reporta en el bloque "odo" de la telemetría. No
// participa del control (el PID de velocidad por rueda es independiente); es
// sólo estimación de estado.
//
// v, x, y: sólo encoders (promedio de rueda y su integración).
// omega/theta (yaw): filtro complementario entre el giroscopio (eje Z, alta
// frecuencia, sin drift de patinaje de rueda) y el yaw derivado de los
// encoders (baja frecuencia, sin drift integral). ODOM_YAW_ALPHA alto =>
// confía más en el giroscopio.
constexpr float ODOM_YAW_ALPHA = 0.10f;

// Calibración de la IMU al arranque (robot QUIETO): promedia estas muestras
// para estimar el bias del giroscopio en Z. Con ~2000 muestras a 2 ms bloquea
// ~4 s, pero la Jetson tarda >10 s en levantar sus nodos, así que hay tiempo
// de sobra antes de recibir comandos.
constexpr uint16_t IMU_CALIB_SAMPLES  = 4000;
constexpr uint16_t IMU_CALIB_DELAY_MS = 2;

// Ventana del filtro de mediana aplicado a gyro.z antes de la media móvil:
// rechaza picos impulsivos (spikes eléctricos/vibración puntual) que una
// media móvil corta deja pasar (e incluso extiende sobre toda la ventana).
// Impar (5): la mediana es siempre el valor central real, sin promediar dos
// muestras del medio como pasaría con una ventana par.
constexpr uint8_t IMU_MEDIAN_WINDOW = 5;

// Ventana de la media móvil aplicada a gyro.z (tras el filtro de mediana)
// para atenuar el ruido de vibración del chasis antes de fusionarlo con los
// encoders. Con TELEMETRY_PERIOD_MS=20ms, una ventana de 8 muestras suaviza
// sobre ~160 ms.
constexpr uint8_t IMU_ROLLING_WINDOW = 8;

// -------- Tipos de mensaje serial (mantener sincronizado con Jetson) --------
namespace MsgType {
    // MOTOR_CMD: Jetson -> ESP32: int16 L, int16 R, int16 aux.
    //   Sólo se usa en modo MANUAL (teleop): L/R = comando crudo de motor
    //   [-CMD_FULL_SCALE, +CMD_FULL_SCALE]. aux: reservado.
    //   En AUTONOMOUS_NAV / AUTONOMOUS_WAYPOINT se ignora: el setpoint de
    //   velocidad llega por VEL_CMD y la cascada/PID on-board es la única
    //   autoridad sobre los motores.
    constexpr uint8_t MOTOR_CMD     = 0x10;
    constexpr uint8_t BRAKE_ON      = 0x11;  // Jetson -> ESP32
    constexpr uint8_t HEARTBEAT     = 0x12;  // Jetson -> ESP32
    constexpr uint8_t PID_PARAM     = 0x13;  // Jetson -> ESP32: ctrl_id(1) param_id(1) float32(4)
    constexpr uint8_t SETPOINT_COMP = 0x14;  // Jetson -> ESP32: comp_id(1) reserved(1) float32(4)
    constexpr uint8_t MODE_CMD      = 0x15;  // Jetson -> ESP32: mode(1) — 0=manual, 1=autonomous nav (PID de velocidad por rueda sobre VEL_CMD)
    // VEL_CMD: Jetson -> ESP32: float32 wheelLeft (rad/s), float32 wheelRight (rad/s).
    //   Setpoint de velocidad por rueda, ya calculado por la Jetson a partir
    //   de /cmd_vel + cinemática diferencial (jetson-bridge). El ESP32 no
    //   hace cinemática ni mezcla lineal/angular: sólo corre un PID de
    //   velocidad independiente por rueda contra el encoder (modo
    //   AUTONOMOUS_NAV). El camino de teleop crudo (MOTOR_CMD) queda
    //   separado.
    constexpr uint8_t VEL_CMD       = 0x16;
    constexpr uint8_t TELEMETRY     = 0x20;  // ESP32 -> Jetson
    constexpr uint8_t ESP_HELLO     = 0x21;  // ESP32 -> Jetson
}

}  // namespace Cfg