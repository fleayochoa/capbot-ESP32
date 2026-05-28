#include "SensorHub.h"
#include "Odometry.h"

#include <ArduinoJson.h>

SensorHub::SensorHub(Capbot::encoderPins encPins, pcnt_unit_t encUnitLeft, 
    pcnt_unit_t encUnitRight, uint16_t filter) :
    encL_(encUnitLeft, encPins.leftA, encPins.leftB, filter),
     encR_(encUnitRight, encPins.rightA, encPins.rightB, filter) {
}

void SensorHub::begin() {
    encL_.begin();
    encR_.begin();
    imu_.begin();
    Serial.println("SensorHub initialized");
}

void SensorHub::sample() {
    last_.enc_left         = encL_.read();
    last_.enc_right        = encR_.read();
    last_.vel_left_cps     = encL_.computeCountsPerSec();
    last_.vel_right_cps    = encR_.computeCountsPerSec();
    last_.imu_accel        = imu_.readAccel();
    last_.imu_gyro         = imu_.readGyro();
    last_.imu_mag          = imu_.readMag();
    imu_.readEuler(last_.imu_euler);
    last_.uptime_ms        = millis();
}

void SensorHub::feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking) {
    last_.motor_pwm_left  = leftPwm;
    last_.motor_pwm_right = rightPwm;
    last_.braking         = braking;
}

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap, const StateEstimate& state, bool autonomousMode, const ControlTelemetry& ctrl) {
    StaticJsonDocument<768> doc;

    
    doc["mode"] = autonomousMode ? "auto" : "manual";
    JsonObject u = doc.createNestedObject("u");
        u["pwm_left"] = last_.motor_pwm_left;
        u["pwm_right"] = last_.motor_pwm_right;

    JsonObject odo = doc.createNestedObject("odo");
    odo["x"]  = state.x;
    odo["y"]  = state.y;
    odo["a"] = state.theta;
    odo["v"]  = state.v;
    odo["w"] = state.omega;

    JsonObject sp = doc.createNestedObject("sp");
    sp["x"] = ctrl.sp_x;
    sp["y"] = ctrl.sp_y;
    sp["a"] = ctrl.sp_ang;
    sp["v"] = ctrl.sp_v;
    sp["w"] = ctrl.sp_w;
    

    const size_t n = serializeJson(doc, out, out_cap);
    if (n == 0 || n >= out_cap) return 0;
    return n;
}