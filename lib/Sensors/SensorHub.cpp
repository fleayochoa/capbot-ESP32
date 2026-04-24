#include "SensorHub.h"

#include <ArduinoJson.h>

SensorHub::SensorHub(Capbot::encoderPins encPins, pcnt_unit_t encUnitLeft, 
    pcnt_unit_t encUnitRight, uint16_t filter) :
    encL_(encUnitLeft, encPins.leftA, encPins.leftB, filter),
     encR_(encUnitRight, encPins.rightA, encPins.rightB, filter) {
}

void SensorHub::begin() {
    encL_.begin();
    encR_.begin();
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

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap) {
    // Documento JSON en stack. Tamaño generoso para los ~12 campos actuales;
    // ajustar si crece.
    StaticJsonDocument<384> doc;

    doc["t_ms"]     = last_.uptime_ms;
    JsonObject enc  = doc.createNestedObject("enc");
    enc["l"]        = last_.enc_left;
    enc["r"]        = last_.enc_right;
    enc["vl_cps"]   = last_.vel_left_cps;
    enc["vr_cps"]   = last_.vel_right_cps;
    enc["err_l"]    = last_.enc_errors_left;
    enc["err_r"]    = last_.enc_errors_right;

    JsonObject mot  = doc.createNestedObject("mot");
    mot["pwm_l"]    = last_.motor_pwm_left;
    mot["pwm_r"]    = last_.motor_pwm_right;
    mot["brake"]    = last_.braking;

    const size_t n = serializeJson(doc, out, out_cap);
    if (n == 0 || n >= out_cap) return 0;
    return n;
}