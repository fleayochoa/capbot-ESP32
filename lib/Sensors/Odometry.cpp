// Odometry.cpp
#include "Odometry.h"
#include <math.h>

Odometry::Odometry(float wheelDiameter, float cpr, float wheelBase)
    : wheelDiameter_(wheelDiameter), cpr_(cpr), wheelBase_(wheelBase),
      lastUpdateMs_(0) {}

void Odometry::begin() {
    reset();
}

void Odometry::reset() {
    state_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lastUpdateMs_ = 0;
}

// Envuelve un ángulo a (-pi, pi].
static float wrapPi(float a) {
    return atan2f(sinf(a), cosf(a));
}

StateEstimate Odometry::update(const SensorHub::Telemetry& telemetry) {
    // Primera muestra: sólo fija el timestamp base, sin integrar.
    if (lastUpdateMs_ == 0) {
        lastUpdateMs_ = telemetry.uptime_ms;
        return state_;
    }

    const float dt = static_cast<float>(telemetry.uptime_ms - lastUpdateMs_) / 1000.0f;
    lastUpdateMs_ = telemetry.uptime_ms;
    if (dt <= 0.0f) return state_;

    // --- Velocidades de rueda desde encoders (m/s) ---
    // cuentas/s -> m/s: (perímetro / cuentas_por_rev).
    const float factor = (PI * wheelDiameter_) / cpr_;
    const float vL = telemetry.vel_left_cps  * factor;
    const float vR = telemetry.vel_right_cps * factor;

    // --- Yaw (theta): filtro complementario gyro-Z + yaw de encoders ---
    // Velocidad angular instantánea: el giroscopio la mide directo y rechaza
    // el patinaje de rueda (que sí contamina el yaw de los encoders).
    const float omegaGyro = telemetry.imu_gyro_z;    // rad/s
    const float omegaEnc  = (vR - vL) / wheelBase_;  // rad/s

    const float thetaGyro = state_.theta + omegaGyro * dt;
    const float thetaEnc  = state_.theta + omegaEnc  * dt;

    if (telemetry.imu_alive) {
        // Integración de ambas fuentes sobre el theta previo, y mezcla: el
        // gyro domina en alta frecuencia; el encoder ancla la baja frecuencia
        // y frena el drift integral del giroscopio.
        state_.omega = omegaGyro;
        state_.theta = wrapPi(Cfg::ODOM_YAW_ALPHA * thetaGyro +
                              (1.0f - Cfg::ODOM_YAW_ALPHA) * thetaEnc);
    } else {
        // IMU no disponible (ver SensorHub::Telemetry::imu_alive): yaw sólo
        // por dead-reckoning de encoders, sin mezcla.
        state_.omega = omegaEnc;
        state_.theta = wrapPi(thetaEnc);
    }

    // --- Velocidad lineal (v): promedio de ambas ruedas (sólo encoders) ---
    state_.v = 0.5f * (vL + vR);

    // --- Posición (x, y): 2ª integración de v proyectada sobre theta ---
    state_.x += state_.v * cosf(state_.theta) * dt;
    state_.y += state_.v * sinf(state_.theta) * dt;

    return state_;
}
