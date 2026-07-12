// Controlador de velocidad por rueda para un robot de tracción diferencial.
//
// Arquitectura de control:
//   Rueda izquierda: PID de velocidad directo sobre el setpoint (rad/s)
//   Rueda derecha  : PID de velocidad directo sobre el setpoint (rad/s)
//
// El setpoint de cada rueda viene de la Jetson (VEL_CMD), ya calculado a
// partir de /cmd_vel + cinemática diferencial (jetson-bridge). No hay
// cinemática, mezcla lineal/angular ni lazo de posición on-board.

#pragma once
#include <Arduino.h>
#include "PID.h"

class Controlador {
public:
    struct Config {
        PidController::Config leftWheelPid;
        PidController::Config rightWheelPid;

        // Límite absoluto para la señal enviada a cada motor
        float maxMotorOutput;

        // Feedforward continuo de fricción por rueda:
        //   u_ff = kStatic * sign(sp) + kV * sp
        // kStatic: esfuerzo para vencer la fricción (offset de Coulomb),
        //          medido en banco (modo MANUAL, rampa de PWM).
        // kV     : pendiente PWM por rad/s en régimen (fricción viscosa +
        //          ganancia del motor), de la misma rampa.
        // A diferencia del "start kick" anterior, este FF se aplica en TODO
        // ciclo con setpoint no nulo, así el esfuerzo no colapsa cuando la
        // rueda rompe a girar (evita stick-slip) y el PID sólo corrige el
        // residuo.
        float leftKStatic;
        float rightKStatic;
        float leftKV;
        float rightKV;
    };

    // Estructura para agrupar las mediciones actuales (realimentación)
    struct State {
        float leftWheelVel;   // rad/s
        float rightWheelVel;  // rad/s
    };

    // Objetivo de control: setpoint de velocidad por rueda (VEL_CMD)
    struct WheelSetpoint {
        float leftWheelVel;   // rad/s
        float rightWheelVel;  // rad/s
    };

    // Salida final mapeada para los drivers de los motores
    struct MotorCommand {
        float left;
        float right;
        bool brake; // Bandera explícita por si se desea invocar motorDriver.brake()
    };

    explicit Controlador(const Config& config);

    // Reinicia las sumatorias integrales y estados previos de todos los PIDs
    void reset();

    // Actualiza las ganancias y parámetros al vuelo
    void setConfig(const Config& config);

    // Control para VEL_CMD: PID de velocidad independiente por rueda, sin
    // cinemática ni lazo de posición on-board.
    MotorCommand computeVelocity(const WheelSetpoint& setpoint, const State& state, float dt);

    const Config& config() const { return config_; }

private:
    Config config_;

    // Instancias de los controladores internos
    PidController leftWheelPid_;
    PidController rightWheelPid_;
};
