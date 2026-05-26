#include "ToFSensors.h"
#include <Wire.h>

ToFSensors::ToFSensors(uint8_t xshut1, uint8_t xshut2)
    : xshut1_(xshut1), xshut2_(xshut2) {}

bool ToFSensors::begin() {
    // Hold both sensors in hardware reset so they don't conflict on 0x29
    pinMode(xshut1_, OUTPUT);
    pinMode(xshut2_, OUTPUT);
    digitalWrite(xshut1_, LOW);
    digitalWrite(xshut2_, LOW);
    delay(10);

    // Boot sensor1, reassign its address before sensor2 comes online
    digitalWrite(xshut1_, HIGH);
    delay(10);
    if (!sensor1_.init()) {
        Serial.println("ToF sensor1 init failed");
        return false;
    }
    sensor1_.setAddress(ADDR_SENSOR1);

    // Boot sensor2; it initializes at the factory default (0x29)
    digitalWrite(xshut2_, HIGH);
    delay(10);
    if (!sensor2_.init()) {
        Serial.println("ToF sensor2 init failed");
        return false;
    }

    sensor1_.setTimeout(100);
    sensor2_.setTimeout(100);
    sensor1_.startContinuous();
    sensor2_.startContinuous();

    ok_ = true;
    Serial.println("ToF sensors initialized");
    return true;
}

uint16_t ToFSensors::readSensor1() {
    if (!ok_) return 0xFFFF;
    return sensor1_.readRangeContinuousMillimeters();
}

uint16_t ToFSensors::readSensor2() {
    if (!ok_) return 0xFFFF;
    return sensor2_.readRangeContinuousMillimeters();
}
