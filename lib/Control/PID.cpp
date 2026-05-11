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
    lastOutput_    = 0.0f;
}

void PidController::setConfig(const Config& config) {
    config_ = config;
    // Acotar la integral actual a los nuevos límites si cambian en caliente
    integralError_ = clampFloat(integralError_, -config_.maxIntegral, config_.maxIntegral);
}

float PidController::compute(float setpoint, float measurement, float dt) {
    // Si el intervalo de tiempo es inválido, retornamos la última salida calculada
    // para evitar divisiones por cero o inestabilidad.
    if (dt <= 0.0f) {
        return lastOutput_;
    }

    // Cálculo del error actual
    const float error = setpoint - measurement;

    // ---- Término Proporcional ----
    const float pTerm = config_.kp * error;

    // ---- Término Integral con Anti-windup (Clamping) ----
    integralError_ += error * dt;
    integralError_ = clampFloat(integralError_, -config_.maxIntegral, config_.maxIntegral);
    const float iTerm = config_.ki * integralError_;

    // ---- Término Derivativo ----
    const float derivative = (error - lastError_) / dt;
    const float dTerm = config_.kd * derivative;

    // Guardamos el error para la siguiente iteración
    lastError_ = error;

    // ---- Salida Total y Saturación ----
    const float output = pTerm + iTerm + dTerm;
    lastOutput_ = clampFloat(output, config_.outMin, config_.outMax);

    return lastOutput_;
}