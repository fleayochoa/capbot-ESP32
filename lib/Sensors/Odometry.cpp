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
    state_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lastUpdateMs_ = 0;
    filterIdx_ = 0;
    filterFull_ = false;
    for (uint8_t i = 0; i < FILTER_WIN; ++i) {
        omegaBuf_[i] = 0.0f;
    }
}

static float windowAvg(const float* buf, uint8_t len) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < len; ++i) sum += buf[i];
    return sum / len;
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

    // Actualización de velocidad lineal y angular
    state_.v = 0.5f * (velLeft + velRight);
    
    // Uso del giroscopio crudo (eje Z) para la velocidad angular (grados/s)
    state_.omega = telemetry.imu_gyro.z;

    // Sliding-window filter for omega
    omegaBuf_[filterIdx_] = state_.omega;
    filterIdx_ = (filterIdx_ + 1) % FILTER_WIN;
    if (!filterFull_ && filterIdx_ == 0) filterFull_ = true;
    uint8_t n = filterFull_ ? FILTER_WIN : filterIdx_;
    state_.omega = windowAvg(omegaBuf_, n);

    // Velocidad angular derivada de la cinemática diferencial (grados/s):
    // omega = (vRight - vLeft) / (2 * distancia rueda-centro)
    float omega_wheels = ((velRight - velLeft) / (2.0f * wheelToCenter_)) * RAD_TO_DEG;

    // Filtro complementario giroscopio + encoders: el giro tiene poca deriva
    // a corto plazo pero acumula sesgo con el tiempo; el odómetro de ruedas
    // no tiene deriva pero es sensible al deslizamiento. Se confía
    // mayormente en el giro y se usa la rueda para corregir el sesgo lento.
    constexpr float kGyroWeight = 0.9f;
    float omega_theta = kGyroWeight * state_.omega + (1.0f - kGyroWeight) * omega_wheels;

    // Heading: integración de la velocidad angular fusionada, normalizada a [-180, 180].
    float thetaRaw = (state_.theta + omega_theta * dt) * DEG_TO_RAD;
    state_.theta = atan2(sin(thetaRaw), cos(thetaRaw)) * RAD_TO_DEG;

    // Integración de la posición (metros)
    float thetaRad = state_.theta * DEG_TO_RAD;
    state_.x += state_.v * cos(thetaRad) * dt;
    state_.y += state_.v * sin(thetaRad) * dt;

    lastUpdateMs_ = telemetry.uptime_ms;
    return state_;
}