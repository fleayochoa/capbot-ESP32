// Controlador jerárquico en cascada para un robot de tracción diferencial.
//
// Arquitectura de control:
//   1. Eje Lineal : PID de Posición Lineal -> Referencia de Vel. Lineal -> PID de Vel. Lineal
//   2. Eje Angular: PID de Orientación Absoluta -> Referencia de Vel. Angular -> PID de Vel. Angular
//
// Lógica de tolerancia (Deadband / Freno):
//   Si el error de posición lineal es menor al umbral especificado (theta),
//   se anula la salida de los motores (salida 0) para frenar de forma segura.

#pragma once
#include <Arduino.h>
#include "PID.h"

class Controlador {
public:
    struct Config {
        // Controladores para el eje lineal
        PidController::Config linearPosPid;
        PidController::Config linearVelPid;

        // Controladores para el eje angular
        PidController::Config angularPosPid;
        PidController::Config angularVelPid;

        // Umbral de tolerancia de error de posición (theta) bajo el cual se aplica freno
        float thetaPositionTolerance;
        float thetaAngleTolerance;

        // Límite absoluto para la señal combinada enviada a cada motor
        float maxMotorOutput;
    };

    // Estructura para agrupar las mediciones actuales (realimentación)
    struct State {
        float linearPosition;
        float linearVelocity;
        float angularPosition; // Orientación absoluta (yaw/theta)
        float angularVelocity;
    };

    // Estructura para los objetivos de control
    struct Setpoint {
        float linearPosition;
        float angularPosition;
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

    // Ejecuta el cálculo completo de la cascada y la mezcla diferencial
    MotorCommand compute(const Setpoint& setpoint, const State& state, float dt);

    const Config& config() const { return config_; }

private:
    Config config_;

    // Instancias de los controladores internos
    PidController linearPosPid_;
    PidController linearVelPid_;
    PidController angularPosPid_;
    PidController angularVelPid_;
};