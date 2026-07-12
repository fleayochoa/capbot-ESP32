// Odometry.h
//
// Estimación de pose/velocidad on-board fusionando encoders + IMU (gyro-Z).
// Se calcula por telemetría (bloque "odo"); NO interviene en el control (el
// PID de velocidad por rueda es independiente).
//
//   - v (velocidad lineal): sólo encoders, promedio de ambas ruedas.
//   - omega / theta (yaw): filtro complementario entre el giroscopio (eje Z)
//     y el yaw derivado de los encoders. Ver Cfg::ODOM_YAW_ALPHA.
//   - x, y: integración de v proyectada sobre theta.
#pragma once

#include <Arduino.h>
#include "SensorHub.h"

struct StateEstimate {
    float x;      // m
    float y;      // m
    float theta;  // rad (yaw), heading absoluto
    float v;      // m/s (velocidad lineal de avance)
    float omega;  // rad/s (velocidad angular)
};

class Odometry {
public:
    // Geometría por defecto: legacy diámetro de rueda + track medido en HW.
    Odometry(float wheelDiameter = Cfg::WHEEL_DIAMETER_M,
             float cpr = Cfg::WHEEL_CPR,
             float wheelBase = Cfg::WHEEL_BASE_M);

    void begin();
    void reset();

    // Fusiona la última telemetría (encoders + IMU) y actualiza el estado.
    StateEstimate update(const SensorHub::Telemetry& telemetry);

    const StateEstimate& state() const { return state_; }

private:
    float wheelDiameter_;
    float cpr_;
    float wheelBase_;
    StateEstimate state_{};
    uint32_t lastUpdateMs_;
};
