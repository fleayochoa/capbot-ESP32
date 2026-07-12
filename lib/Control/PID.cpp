#include "PID.h"

// ---- Helpers ----
static inline float clampFloat(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---- Inicialización de miembros ----
PidController::PidController(const Config& config)
    : config_(config) {
    reset();
}

// ---- API pública ----
void PidController::reset() {
    integralError_ = 0.0f;
    lastError_     = 0.0f;
    firstCompute_  = true;
    lastOutput_    = 0.0f;
}

void PidController::setConfig(const Config& config) {
    config_ = config;
    // Acotar la integral actual a los nuevos límites si cambian en caliente
    integralError_ = clampFloat(integralError_, -config_.maxIntegral, config_.maxIntegral);
}

float PidController::compute(float error, float dt) {
    // Si el intervalo de tiempo es inválido, retornamos la última salida calculada
    // para evitar divisiones por cero o inestabilidad.
    if (dt <= 0.0f) {
        return lastOutput_;
    }

    // ---- Término Proporcional ----
    const float pTerm = config_.kp * error;

    // ---- Término Integral con Anti-windup (Clamping) ----
    integralError_ += error * dt;
    integralError_ = clampFloat(integralError_, -config_.maxIntegral, config_.maxIntegral);
    const float iTerm = config_.ki * integralError_;

    // ---- Término Derivativo ----
    // Primer ciclo tras reset(): no hay error previo válido, derivada = 0
    // para no inyectar un pico espurio al (re)entrar al lazo.
    const float derivative = firstCompute_ ? 0.0f : (error - lastError_) / dt;
    const float dTerm = config_.kd * derivative;

    lastError_    = error;
    firstCompute_ = false;

    // ---- Salida Total y Saturación ----
    lastOutput_ = clampFloat(pTerm + iTerm + dTerm, config_.outMin, config_.outMax);

    return lastOutput_;
}
