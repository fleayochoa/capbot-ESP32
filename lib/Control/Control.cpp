#include "Control.h"

// ---- Helpers ----
static inline float clampFloat(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---- Inicialización de miembros ----
Controlador::Controlador(const Config& config)
    : config_(config),
      linearVelPid_(config.linearVelPid),
      angularVelPid_(config.angularVelPid) {
}

// ---- API pública ----
void Controlador::reset() {
    linearVelPid_.reset();
    angularVelPid_.reset();
}

void Controlador::setConfig(const Config& config) {
    config_ = config;
    linearVelPid_.setConfig(config_.linearVelPid);
    angularVelPid_.setConfig(config_.angularVelPid);
}

Controlador::MotorCommand Controlador::computeVelocity(
    const VelSetpoint& setpoint, const State& state, float dt) {

    MotorCommand cmd = {0.0f, 0.0f, false, setpoint.linearVelocity, setpoint.angularVelocity};

    if (dt <= 0.0f) {
        return cmd;
    }

    // Sin lazo de posición: el setpoint de velocidad viene directo de nav2.
    const float linearVelError  = setpoint.linearVelocity  - state.linearVelocity;
    const float angularVelError = setpoint.angularVelocity - state.angularVelocity;

    const float linearEffort  = linearVelPid_.compute(linearVelError, dt);
    const float angularEffort = angularVelPid_.compute(angularVelError, dt);

    // ---- Mezclador (Cinemática Inversa Diferencial) ----
    const float leftOut  = linearEffort - angularEffort;
    const float rightOut = linearEffort + angularEffort;

    cmd.left  = clampFloat(leftOut,  -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.right = clampFloat(rightOut, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.brake = false;

    return cmd;
}
