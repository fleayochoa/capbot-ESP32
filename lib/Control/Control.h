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

        // Feedforward de arranque (fricción estática): esfuerzo que se suma
        // al output del PID sólo en el instante de arrancar desde parado
        // (ver kStationaryThreshold en Control.cpp), para vencer la fricción
        // estática sin depender de un Kp desproporcionado. Medido en banco
        // (modo MANUAL, rampa de PWM) como el duty mínimo al que la rueda
        // rompe a girar.
        float leftStartPwm;
        float rightStartPwm;
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
