// Controlador de velocidad para un robot de tracción diferencial.
//
// Arquitectura de control:
//   nav2 (/cmd_vel) da un setpoint de velocidad lineal/angular; un PID por
//   eje cierra el lazo contra la velocidad medida (odometría de ruedas) y
//   entrega el esfuerzo diferencial para cada motor.
//
// No hay lazo de posición on-board: la navegación (planificación, corrección
// de pose) vive en nav2 + EKF, en la Jetson.

#pragma once
#include <Arduino.h>
#include "PID.h"

class Controlador {
public:
    struct Config {
        // Controladores de velocidad (lineal y angular)
        PidController::Config linearVelPid;
        PidController::Config angularVelPid;

        // Límite absoluto para la señal combinada enviada a cada motor
        float maxMotorOutput;
    };

    // Estructura para agrupar las mediciones actuales (realimentación)
    struct State {
        float linearVelocity;
        float angularVelocity;
    };

    // Objetivo de control en modo velocidad directa (nav2 /cmd_vel)
    struct VelSetpoint {
        float linearVelocity;   // m/s
        float angularVelocity;  // grados/s (mismo eje que State.angularVelocity)
    };

    // Salida final mapeada para los drivers de los motores
    struct MotorCommand {
        float left;
        float right;
        bool brake; // Bandera explícita por si se desea invocar motorDriver.brake()
        float targetLinVel;
        float targetAngVel;
    };

    explicit Controlador(const Config& config);

    // Reinicia las sumatorias integrales y estados previos de todos los PIDs
    void reset();

    // Actualiza las ganancias y parámetros al vuelo
    void setConfig(const Config& config);

    // Control de velocidad directo para nav2 /cmd_vel: PID de velocidad
    // lineal y angular sobre el setpoint recibido.
    MotorCommand computeVelocity(const VelSetpoint& setpoint, const State& state, float dt);

    const Config& config() const { return config_; }

private:
    Config config_;

    // Instancias de los controladores internos
    PidController linearVelPid_;
    PidController angularVelPid_;
};
