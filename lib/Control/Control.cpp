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
    
    MotorCommand cmd = {0.0f, 0.0f, false, 0.0f, 0.0f};

    if (dt <= 0.0f) {
        return cmd;
    }

    // Cálculo de errores en posición
    const float linearPosError  = sqrt((setpoint.xPosition - state.xPosition) * (setpoint.xPosition - state.xPosition) +
                                      (setpoint.yPosition - state.yPosition) *
                                      (setpoint.yPosition - state.yPosition));
    const float angularPosError = atan2(setpoint.yPosition - state.yPosition, setpoint.xPosition - state.xPosition) * RAD_TO_DEG
                                        - state.angularPosition;

 
    // ---- Lazo Lineal en Cascada ----
    // 1. El PID de posición lineal calcula la velocidad lineal requerida
    float targetAngularVel = 0.0f;
    float angularEffort = 0.0f;
    float targetLinearVel = 0.0f;
    float linearEffort = 0.0f;


    if (absFloat(linearPosError) < config_.thetaPositionTolerance) {
        // Si estamos dentro de la tolerancia de posición, llegamos
        linearVelPid_.reset();
        angularVelPid_.reset();
        
        cmd.left  = 0.0f;
        cmd.right = 0.0f;
        cmd.brake = true;
        return cmd;
    } else {
        // Si el error de posición es grande, permitimos que el PID de posición lineal
        // genere una velocidad que el PID de velocidad intentará alcanzar.
    targetLinearVel = linearPosPid_.compute(linearPosError, dt);
    
    const float linearVelError = targetLinearVel - state.linearVelocity;
    // 2. El PID de velocidad lineal genera el esfuerzo de tracción base (ej. PWM o Fuerza)
    linearEffort = linearVelPid_.compute(linearVelError, dt);
    }
    // ---- Lazo Angular en Cascada ----
    // 1. El PID de orientación absoluta calcula la velocidad angular requerida
    if (absFloat(angularPosError) < config_.thetaAngleTolerance) {
        // Si estamos dentro de la tolerancia angular, no corregimos la orientación
        // para evitar oscilaciones finas. Solo corregimos la posición lineal.
        angularPosPid_.reset();
        angularVelPid_.reset();
        targetAngularVel = 0.0f;
        angularEffort = 0.0f;
    } else {
        // Si el error angular es grande, permitimos que el PID de posición angular
        // genere una velocidad de giro que el PID de velocidad angular intentará alcanzar.
        targetAngularVel = angularPosPid_.compute(angularPosError, dt);
        const float angularVelError = targetAngularVel - state.angularVelocity;
        // 2. El PID de velocidad angular genera el esfuerzo de giro diferencial
        angularEffort = angularVelPid_.compute(angularVelError, dt);
    }
    

    // ---- Mezclador (Cinemática Inversa Diferencial) ----
    // Rueda izquierda = Esfuerzo lineal - Esfuerzo angular
    // Rueda derecha   = Esfuerzo lineal + Esfuerzo angular
    const float leftOut  = linearEffort - angularEffort;
    const float rightOut = linearEffort + angularEffort;

    // Saturación final de los comandos aplicados a los motores
    cmd.left  = clampFloat(leftOut,  -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.right = clampFloat(rightOut, -config_.maxMotorOutput, config_.maxMotorOutput);
    cmd.brake = false;
    cmd.targetLinVel = targetLinearVel;
    cmd.targetAngVel = targetAngularVel;

    return cmd;
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