#include "Control.h"

// ---- Helpers ----
static inline float clampFloat(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Feedforward de fricción: u_ff = kStatic*sign(sp) + kV*sp.
// Bajo kMinMoveSetpoint el FF es 0: un setpoint ~0 se maneja sólo con el
// PID, sin dejar un offset de fricción empujando al robot detenido.
static inline float frictionFeedforward(float sp, float kStatic, float kV) {
    constexpr float kMinMoveSetpoint = 0.05f;  // rad/s
    if (sp > kMinMoveSetpoint)  return  kStatic + kV * sp;
    if (sp < -kMinMoveSetpoint) return -kStatic + kV * sp;
    return 0.0f;
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

    // Feedforward continuo + PID: el FF aporta el esfuerzo estimado para
    // vencer la fricción y sostener el setpoint; el PID corrige el residuo.
    const float leftFF  = frictionFeedforward(setpoint.leftWheelVel,
                                              config_.leftKStatic,  config_.leftKV);
    const float rightFF = frictionFeedforward(setpoint.rightWheelVel,
                                              config_.rightKStatic, config_.rightKV);

    float leftEffort  = leftFF  + leftWheelPid_.compute(leftError, dt);
    float rightEffort = rightFF + rightWheelPid_.compute(rightError, dt);

    cmd.left  = clampFloat(leftEffort,  -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.right = clampFloat(rightEffort, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.brake = false;

    return cmd;
}
