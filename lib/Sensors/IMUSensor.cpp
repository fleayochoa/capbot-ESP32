#include "IMUSensor.h"

#include <math.h>

// Factor de conversión rad/s -> deg/s (Adafruit expone gyro en rad/s).
static constexpr float RPS_TO_DPS = 57.29577951308232f;   // 180/π

IMUSensor::IMUSensor(uint8_t addr, TwoWire& wire)
    : addr_(addr),
      wire_(&wire) {}

bool IMUSensor::begin() {
    if (!mpu_.begin(addr_, wire_)) {
        return false;
    }

    mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // I2C a 400 kHz (Adafruit deja el default de Wire, típicamente 100 kHz).
    wire_->setClock(400000);

    return true;
}

bool IMUSensor::isConnected() {
    // Ping I2C crudo (la lib de Adafruit no expone esto). Si el chip no
    // contesta el ACK a la dirección, endTransmission() != 0.
    wire_->beginTransmission(addr_);
    return wire_->endTransmission() == 0;
}

void IMUSensor::calibrate(uint16_t samples) {
    if (samples == 0) return;

    float accelSumX = 0.0f, accelSumY = 0.0f, accelSumZ = 0.0f;
    float gyroSumX  = 0.0f, gyroSumY  = 0.0f, gyroSumZ  = 0.0f;

    for (uint16_t i = 0; i < samples; ++i) {
        sensors_event_t accelEvt, gyroEvt, tempEvt;
        mpu_.getEvent(&accelEvt, &gyroEvt, &tempEvt);

        accelSumX += accelEvt.acceleration.x;
        accelSumY += accelEvt.acceleration.y;
        accelSumZ += accelEvt.acceleration.z;

        gyroSumX += gyroEvt.gyro.x * RPS_TO_DPS;
        gyroSumY += gyroEvt.gyro.y * RPS_TO_DPS;
        gyroSumZ += gyroEvt.gyro.z * RPS_TO_DPS;

        delay(5);
    }

    accelBias_.x = accelSumX / samples;
    accelBias_.y = accelSumY / samples;
    accelBias_.z = accelSumZ / samples;

    gyroBias_.x = gyroSumX / samples;
    gyroBias_.y = gyroSumY / samples;
    gyroBias_.z = gyroSumZ / samples;
}

// ================= Lecturas =================

IMUSensor::Vec3 IMUSensor::readAccel() {
    sensors_event_t accelEvt, gyroEvt, tempEvt;
    mpu_.getEvent(&accelEvt, &gyroEvt, &tempEvt);

    return Vec3{
        accelEvt.acceleration.x - accelBias_.x,
        accelEvt.acceleration.y - accelBias_.y,
        accelEvt.acceleration.z - accelBias_.z
    };
}

IMUSensor::Vec3 IMUSensor::readGyro() {
    sensors_event_t accelEvt, gyroEvt, tempEvt;
    mpu_.getEvent(&accelEvt, &gyroEvt, &tempEvt);

    return Vec3{
        gyroEvt.gyro.x * RPS_TO_DPS - gyroBias_.x,
        gyroEvt.gyro.y * RPS_TO_DPS - gyroBias_.y,
        gyroEvt.gyro.z * RPS_TO_DPS - gyroBias_.z
    };
}
