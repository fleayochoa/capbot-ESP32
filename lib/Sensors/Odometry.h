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
    StateEstimate update(const SensorHub::Telemetry& telemetry, bool useInternalFusion);
    
    void reset();

    const StateEstimate& state() const { return state_; }

private:
    static constexpr uint8_t FILTER_WIN = 5;

    float wheelDiameter_;
    float cpr_;
    float lastOmegas[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // buffer para filtro de media móvil
    StateEstimate state_{};
    uint32_t lastUpdateMs_;

    float thetaBuf_[FILTER_WIN]{};
    float omegaBuf_[FILTER_WIN]{};
    uint8_t filterIdx_ = 0;
    bool filterFull_ = false;
};