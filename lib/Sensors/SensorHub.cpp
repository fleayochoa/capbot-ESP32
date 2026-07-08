#include "SensorHub.h"
#include "Odometry.h"

#include <ArduinoJson.h>

SensorHub::SensorHub(Capbot::encoderPins encPins, pcnt_unit_t encUnitLeft,
    pcnt_unit_t encUnitRight, uint8_t tofXshut1, uint8_t tofXshut2,
    uint16_t filter) :
    encL_(encUnitLeft, encPins.leftA, encPins.leftB, filter),
    encR_(encUnitRight, encPins.rightA, encPins.rightB, filter),
    tof_(tofXshut1, tofXshut2) {
}

void SensorHub::begin() {
    encL_.begin();
    encR_.begin();
    tof_.begin();
    Serial.println("SensorHub initialized");
}

void SensorHub::sample() {
    last_.enc_left         = encL_.read();
    last_.enc_right        = encR_.read();
    last_.vel_left_cps     = encL_.computeCountsPerSec();
    last_.vel_right_cps    = encR_.computeCountsPerSec();
    last_.tof_sensor1_mm   = tof_.readSensor1();
    last_.tof_sensor2_mm   = tof_.readSensor2();
    last_.uptime_ms        = millis();
}

void SensorHub::feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking) {
    last_.motor_pwm_left  = leftPwm;
    last_.motor_pwm_right = rightPwm;
    last_.braking         = braking;
}

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap, const StateEstimate& state, const char* mode, const ControlTelemetry& ctrl) {
    StaticJsonDocument<768> doc;

    doc["mode"] = mode;
    JsonObject u = doc.createNestedObject("u");
        u["pwm_left"] = last_.motor_pwm_left;
        u["pwm_right"] = last_.motor_pwm_right;

    // v/omega crudos de cinemática de ruedas (sin IMU, sin filtrado): la
    // integración de pose y cualquier suavizado quedan del lado de la Jetson
    // (EKF), no aquí.
    JsonObject odo = doc.createNestedObject("odo");
    odo["v"] = state.v;
    odo["w"] = state.omega;

    JsonObject sp = doc.createNestedObject("sp");
    sp["v"] = ctrl.sp_v;
    sp["w"] = ctrl.sp_w;

    JsonObject tof = doc.createNestedObject("tof");
    tof["t1"]  = last_.tof_sensor1_mm;
    tof["t2"]  = last_.tof_sensor2_mm;

    const size_t n = serializeJson(doc, out, out_cap);
    if (n == 0 || n >= out_cap) return 0;
    return n;
}