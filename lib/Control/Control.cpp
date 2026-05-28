#include "Control.h"

// ---- Helpers ----
static inline float clampFloat(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float absFloat(float v) {
    return (v < 0.0f) ? -v : v;
}

// ---- Inicialización de miembros ----
Controlador::Controlador(const Config& config)
    : config_(config),
      linearPosPid_(config.linearPosPid),
      linearVelPid_(config.linearVelPid),
      angularPosPid_(config.angularPosPid),
      angularVelPid_(config.angularVelPid) {
}

// ---- API pública ----
void Controlador::reset() {
    linearPosPid_.reset();
    linearVelPid_.reset();
    angularPosPid_.reset();
    angularVelPid_.reset();
}

void Controlador::setConfig(const Config& config) {
    config_ = config;
    linearPosPid_.setConfig(config_.linearPosPid);
    linearVelPid_.setConfig(config_.linearVelPid);
    angularPosPid_.setConfig(config_.angularPosPid);
    angularVelPid_.setConfig(config_.angularVelPid);
}

Controlador::MotorCommand Controlador::compute(
    const Setpoint& setpoint, const State& state, float dt) {
    
    MotorCommand cmd = {0.0f, 0.0f, false};

    if (dt <= 0.0f) {
        return cmd;
    }

    // Cálculo de errores en posición
    const float linearPosError  = setpoint.linearPosition - state.linearPosition;
    const float angularPosError = setpoint.angularPosition - state.angularPosition;

    // ---- Condición de Parada / Frenado (Tolerancia Theta) ----
    // Si el error posicional cae dentro de la tolerancia, forzamos salida 0 y freno.
    if (absFloat(linearPosError) < config_.thetaPositionTolerance &&
        absFloat(angularPosError) < config_.thetaAngleTolerance) {
        
        // Se reinician los integradores de velocidad para evitar arranques bruscos
        // por acumulación previa (windup) si el setpoint vuelve a cambiar.
        linearVelPid_.reset();
        angularVelPid_.reset();
        
        cmd.left  = 0.0f;
        cmd.right = 0.0f;
        cmd.brake = true;
        return cmd;
    }

    // ---- Lazo Lineal en Cascada ----
    // 1. El PID de posición lineal calcula la velocidad lineal requerida
    const float targetLinearVel = linearPosPid_.compute(setpoint.linearPosition, state.linearPosition, dt);
    
    // 2. El PID de velocidad lineal genera el esfuerzo de tracción base (ej. PWM o Fuerza)
    const float linearEffort = linearVelPid_.compute(targetLinearVel, state.linearVelocity, dt);

    // ---- Lazo Angular en Cascada ----
    // 1. El PID de orientación absoluta calcula la velocidad angular requerida
    const float targetAngularVel = angularPosPid_.compute(setpoint.angularPosition, state.angularPosition, dt);
    
    // 2. El PID de velocidad angular genera el esfuerzo de giro diferencial
    const float angularEffort = angularVelPid_.compute(targetAngularVel, state.angularVelocity, dt);

    // ---- Mezclador (Cinemática Inversa Diferencial) ----
    // Rueda izquierda = Esfuerzo lineal - Esfuerzo angular
    // Rueda derecha   = Esfuerzo lineal + Esfuerzo angular
    const float leftOut  = linearEffort - angularEffort;
    const float rightOut = linearEffort + angularEffort;

    // Saturación final de los comandos aplicados a los motores
    cmd.left  = clampFloat(leftOut,  -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.right = clampFloat(rightOut, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.brake = false;

    return cmd;
}

Controlador::MotorCommand Controlador::computeVelocity(
    const VelSetpoint& setpoint, const State& state, float dt) {

    MotorCommand cmd = {0.0f, 0.0f, false};
    if (dt <= 0.0f) return cmd;

    const float linearEffort  = linearVelPid_.compute(setpoint.linearVelocity,  state.linearVelocity,  dt);
    const float angularEffort = angularVelPid_.compute(setpoint.angularVelocity, state.angularVelocity, dt);

    cmd.left  = clampFloat(linearEffort - angularEffort, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.right = clampFloat(linearEffort + angularEffort, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.brake = false;
    return cmd;
}