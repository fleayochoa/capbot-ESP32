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
    last_.imu_linear_accel = imu_.readLinearAccel();
    last_.imu_orientation  = imu_.readOrientation();
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

    doc["t"]     = last_.uptime_ms;
    /*JsonObject enc  = doc.createNestedObject("enc");
    enc["l"]        = last_.enc_left;
    enc["r"]        = last_.enc_right;
    enc["lc"]   = last_.vel_left_cps;
    enc["rc"]   = last_.vel_right_cps;

    JsonObject mot  = doc.createNestedObject("mot");
    mot["pl"]    = last_.motor_pwm_left;
    mot["pr"]    = last_.motor_pwm_right;
    mot["brk"]    = last_.braking;*/

    JsonObject imu  = doc.createNestedObject("imu");
    /*imu["mx"]    = last_.imu_mag.x;
    imu["my"]    = last_.imu_mag.y;
    imu["mz"]    = last_.imu_mag.z;
    imu["ax"]    = last_.imu_accel.x;
    imu["ay"]    = last_.imu_accel.y;
    imu["az"]    = last_.imu_accel.z;
    imu["gx"]    = last_.imu_gyro.x;*/
    imu["gy"]    = last_.imu_gyro.y;
    imu["gz"]    = last_.imu_gyro.z;
    imu["lax"]   = last_.imu_linear_accel.x;
    imu["lay"]   = last_.imu_linear_accel.y;
    imu["laz"]   = last_.imu_linear_accel.z;
    imu["ori"]   = last_.imu_orientation;
    
        // Serializamos el JSON al buffer de salida. Si no entra o hay error, devolvemos 0.

    const size_t n = serializeJson(doc, out, out_cap);
    if (n == 0 || n >= out_cap) return 0;
    return n;
}