#include "SensorHub.h"

#include <ArduinoJson.h>

SensorHub::SensorHub(Capbot::encoderPins encPins, pcnt_unit_t encUnitLeft,
    pcnt_unit_t encUnitRight,
    uint16_t filter) :
    encL_(encUnitLeft, encPins.leftA, encPins.leftB, filter),
    encR_(encUnitRight, encPins.rightA, encPins.rightB, filter)
     {
}

void SensorHub::begin() {
    encL_.begin();
    encR_.begin();
    Serial.println("SensorHub initialized");
}

void SensorHub::sample() {
    last_.enc_left         = encL_.read();
    last_.enc_right        = encR_.read();
    last_.vel_left_cps     = encL_.computeCountsPerSec();
    last_.vel_right_cps    = encR_.computeCountsPerSec();
    last_.uptime_ms        = millis();
}

void SensorHub::feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking) {
    last_.motor_pwm_left  = leftPwm;
    last_.motor_pwm_right = rightPwm;
    last_.braking         = braking;
}

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap, const char* mode, const ControlTelemetry& ctrl) {
    StaticJsonDocument<768> doc;

    doc["mode"] = mode;
    JsonObject u = doc.createNestedObject("u");
        u["enc_left"]  = last_.enc_left;
        u["enc_right"] = last_.enc_right;
        u["vel_left_cps"]  = last_.vel_left_cps;
        u["vel_right_cps"] = last_.vel_right_cps;
        u["pwm_left"]  = last_.motor_pwm_left;
        u["pwm_right"] = last_.motor_pwm_right;
        u["braking"]   = last_.braking;

    JsonObject c = doc.createNestedObject("ctrl");
        c["sp_left"]  = ctrl.sp_left;
        c["sp_right"] = ctrl.sp_right;

    const size_t n = serializeJson(doc, out, out_cap);
    if (n == 0 || n >= out_cap) return 0;
    return n;
}