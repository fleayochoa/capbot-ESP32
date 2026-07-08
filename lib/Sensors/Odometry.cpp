// Odometry.cpp
#include "Odometry.h"
#include <math.h>


Odometry::Odometry(float wheelDiameter, float cpr, float wheelToCenter)
    : wheelDiameter_(wheelDiameter), cpr_(cpr), wheelToCenter_(wheelToCenter), lastUpdateMs_(0) {
}

void Odometry::begin() {
    reset();
}

void Odometry::reset() {
    state_ = {0.0f, 0.0f};
    lastUpdateMs_ = 0;
}

StateEstimate Odometry::update(const SensorHub::Telemetry& telemetry) {
    if (lastUpdateMs_ == 0) {
        lastUpdateMs_ = telemetry.uptime_ms;
        return state_;
    }

    float dt = static_cast<float>(telemetry.uptime_ms - lastUpdateMs_) / 1000.0f;
    if (dt <= 0.0f) return state_;

    // Factor de conversión: de cuentas por segundo a metros por segundo
    const float factor = (PI * wheelDiameter_) / cpr_;

    // Velocidades lineales de cada rueda (m/s)
    // Se mantiene la inversión de signo en la rueda izquierda de la lógica original
    float velLeft  = telemetry.vel_left_cps * factor;
    float velRight =  telemetry.vel_right_cps * factor;

    // Velocidad lineal: promedio de ambas ruedas.
    state_.v = 0.5f * (velLeft + velRight);

    // Velocidad angular: cinemática diferencial pura (sin IMU/gyro).
    // omega = (vRight - vLeft) / (2 * distancia rueda-centro)
    state_.omega = ((velRight - velLeft) / (2.0f * wheelToCenter_)) * RAD_TO_DEG;

    lastUpdateMs_ = telemetry.uptime_ms;
    return state_;
}
