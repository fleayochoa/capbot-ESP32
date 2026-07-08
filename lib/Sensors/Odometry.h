// Odometry.h
#pragma once

#include <Arduino.h>
#include "SensorHub.h"

// Estado cinemático crudo del robot, derivado únicamente de encoders
// (sin IMU, sin integración de pose). La pose (x,y,theta) se integra
// aguas arriba, en la Jetson (EKF), a partir de v/omega.
struct StateEstimate {
    float v;      // m/s
    float omega;  // grados/s
};

class Odometry {
public:
    // Valores por defecto basados en la configuración legacy.
    // wheelToCenter: distancia de cada rueda al centro del vehículo (m).
    Odometry(float wheelDiameter = 0.067f, float cpr = 910.0f, float wheelToCenter = 0.086f);

    void begin();

    // Calcula v/omega instantáneos (cinemática diferencial pura) a partir
    // de la telemetría agregada del SensorHub. Sin filtrado ni fusión.
    StateEstimate update(const SensorHub::Telemetry& telemetry);

    void reset();

    const StateEstimate& state() const { return state_; }

private:
    float wheelDiameter_;
    float cpr_;
    float wheelToCenter_;
    StateEstimate state_{};
    uint32_t lastUpdateMs_;
};
