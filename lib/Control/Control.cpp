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
      leftWheelPid_(config.leftWheelPid),
      rightWheelPid_(config.rightWheelPid) {
}

// ---- API pública ----
void Controlador::reset() {
    leftWheelPid_.reset();
    rightWheelPid_.reset();
}

void Controlador::setConfig(const Config& config) {
    config_ = config;
    leftWheelPid_.setConfig(config_.leftWheelPid);
    rightWheelPid_.setConfig(config_.rightWheelPid);
}

Controlador::MotorCommand Controlador::computeVelocity(
    const WheelSetpoint& setpoint, const State& state, float dt) {

    MotorCommand cmd = {0.0f, 0.0f, false};

    if (dt <= 0.0f) {
        return cmd;
    }

    const float leftError  = setpoint.leftWheelVel  - state.leftWheelVel;
    const float rightError = setpoint.rightWheelVel - state.rightWheelVel;

    float leftEffort  = leftWheelPid_.compute(leftError, dt);
    float rightEffort = rightWheelPid_.compute(rightError, dt);

    // Feedforward de arranque: sólo se aplica en el instante de partir desde
    // parado (rueda ya en movimiento -> no se suma, para no sumar un empujón
    // fijo en cada ciclo y desestabilizar el lazo cerrado).
    constexpr float kStationaryThreshold = 0.05f;  // rad/s
    constexpr float kMinMoveSetpoint     = 0.05f;  // rad/s

    if (state.leftWheelVel > -kStationaryThreshold && state.leftWheelVel < kStationaryThreshold) {
        if (setpoint.leftWheelVel > kMinMoveSetpoint) {
            leftEffort += config_.leftStartPwm;
        } else if (setpoint.leftWheelVel < -kMinMoveSetpoint) {
            leftEffort -= config_.leftStartPwm;
        }
    }
    if (state.rightWheelVel > -kStationaryThreshold && state.rightWheelVel < kStationaryThreshold) {
        if (setpoint.rightWheelVel > kMinMoveSetpoint) {
            rightEffort += config_.rightStartPwm;
        } else if (setpoint.rightWheelVel < -kMinMoveSetpoint) {
            rightEffort -= config_.rightStartPwm;
        }
    }

    cmd.left  = clampFloat(leftEffort,  -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.right = clampFloat(rightEffort, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.brake = false;

    return cmd;
}
