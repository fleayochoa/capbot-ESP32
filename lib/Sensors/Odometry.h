// Odometry.h
#pragma once

#include <Arduino.h>
#include "SensorHub.h"

struct StateEstimate {
    float x;
    float y;
    float theta;  // grados
    float v;      // m/s
    float omega;  // grados/s
};

class Odometry {
public:
    // Valores por defecto basados en la configuración legacy
    Odometry(float wheelDiameter = 0.067f, float cpr = 910.0f);

    void begin();
    
    // Calcula la cinemática usando la telemetría agregada del SensorHub
    void update(const SensorHub::Telemetry& telemetry, bool useInternalFusion);
    
    void reset();

    const StateEstimate& state() const { return state_; }

private:
    float wheelDiameter_;
    float cpr_;
    StateEstimate state_{};
    uint32_t lastUpdateMs_;
};