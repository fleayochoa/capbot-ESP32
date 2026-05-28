// Odometry.cpp
#include "Odometry.h"
#include <math.h>


Odometry::Odometry(float wheelDiameter, float cpr)
    : wheelDiameter_(wheelDiameter), cpr_(cpr), lastUpdateMs_(0) {
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
        thetaBuf_[i] = 0.0f;
        omegaBuf_[i] = 0.0f;
    }
}

static float windowAvg(const float* buf, uint8_t len) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < len; ++i) sum += buf[i];
    return sum / len;
}

void Odometry::update(const SensorHub::Telemetry& telemetry, bool useInternalFusion) {
    float alpha = 0.98;

    if (lastUpdateMs_ == 0) {
        lastUpdateMs_ = telemetry.uptime_ms;
        return;
    }

    float dt = static_cast<float>(telemetry.uptime_ms - lastUpdateMs_) / 1000.0f;
    if (dt <= 0.0f) return;

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

    // Uso de la fusión interna (grados)
    if (useInternalFusion) {
        state_.theta = telemetry.imu_euler.heading;
    }
    // Filtro complementario
    else {
        float theta_gyro = state_.theta + state_.omega * dt;
        float theta_mag;
        if (telemetry.imu_mag.x == 0) {
            if (telemetry.imu_mag.y > 0) {
                theta_mag = 90.0;
            }
            else if (telemetry.imu_mag.y < 0) {
                theta_mag = 90.0;
            }
        }
        else {
            theta_mag = atan2(telemetry.imu_mag.x,telemetry.imu_mag.y) * RAD_TO_DEG;
        }
        state_.theta = alpha * theta_gyro + (1 - alpha) * theta_mag;
    }
    // Sliding-window filter for theta and omega
    thetaBuf_[filterIdx_] = state_.theta;
    omegaBuf_[filterIdx_] = state_.omega;
    filterIdx_ = (filterIdx_ + 1) % FILTER_WIN;
    if (!filterFull_ && filterIdx_ == 0) filterFull_ = true;
    uint8_t n = filterFull_ ? FILTER_WIN : filterIdx_;
    state_.theta = windowAvg(thetaBuf_, n);
    state_.omega = windowAvg(omegaBuf_, n);

    // Integración de la posición (metros)
    float thetaRad = state_.theta * DEG_TO_RAD;
    state_.x += state_.v * cos(thetaRad) * dt;
    state_.y += state_.v * sin(thetaRad) * dt;

    lastUpdateMs_ = telemetry.uptime_ms;
}