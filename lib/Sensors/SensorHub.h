// Agregador de sensores. Hoy maneja dos encoders, pero la idea es que
// crezca (IMU, voltaje de batería, corrientes, temperaturas, etc.) sin
// cambiar el código que lo consume.
//
// La telemetría se serializa en JSON UTF-8 usando ArduinoJson. Esto permite
// que la Jetson la vuelque directamente al WebSocket sin re-parsear. Cuando
// el throughput empiece a apretar, se puede migrar a un struct binario
// (cambiando solo buildPayload y el decoder de la Jetson).

#pragma once
#include <Arduino.h>
#include <stdint.h>

#include "Config.h"
#include "QuadratureEncoder.h"
#include "CapTypes.h"
#include "IMUSensor.h"

// Definido en Odometry.h; SensorHub sólo necesita el tipo por referencia para
// serializarlo en buildPayload (evita el ciclo de include con Odometry.h).
struct StateEstimate;

class SensorHub {
public:

    SensorHub(Capbot::encoderPins encPins, pcnt_unit_t encUnitLeft,
         pcnt_unit_t encUnitRight,
         uint16_t filter = 100);

    struct Telemetry {
        int32_t enc_left;
        int32_t enc_right;
        float   vel_left_cps;   // cuentas por segundo
        float   vel_right_cps;
        int16_t motor_pwm_left;
        int16_t motor_pwm_right;
        bool    braking;
        float   imu_gyro_z;  // rad/s (yaw rate, ver IMUSensor)
        uint32_t uptime_ms;
    };

    // Setpoint de velocidad por rueda vigente (VEL_CMD), para telemetría.
    struct ControlTelemetry {
        float sp_left;   // rad/s
        float sp_right;  // rad/s
    };

    // Configura los encoders. Llamar en setup().
    void begin();

    // Actualiza muestras cacheadas (velocidades y contadores). Llamar
    // periódicamente (ej. cada tick de telemetría) antes de buildPayload.
    void sample();

    // Inyecta info del estado del motor (duty y freno) que viene del
    // MotorDriver. Así el payload es autocontenido.
    void feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking);

    // Serializa la telemetría a JSON en el buffer dado. Devuelve bytes
    // escritos (sin NUL final) o 0 en error. mode: "manual" | "nav2".
    // state: pose/velocidad estimada por la odometría on-board (bloque "odo").
    size_t buildPayload(uint8_t* out, size_t out_cap, const StateEstimate& state,
                        const char* mode, const ControlTelemetry& ctrl);

    const Telemetry& last() const { return last_; }

private:
    QuadratureEncoder encL_;
    QuadratureEncoder encR_;
    IMUSensor imu_;
    Telemetry last_{};
};