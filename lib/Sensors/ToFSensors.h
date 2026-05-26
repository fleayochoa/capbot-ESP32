#pragma once
#include <Arduino.h>
#include <VL53L0X.h>

// Manages two VL53L0X ToF sensors on a shared I2C bus.
// At startup both XSHUT pins are driven low to hold sensors in reset, then
// sensor1 is booted alone and reassigned to ADDR_SENSOR1, then sensor2 is
// booted (stays at the factory default 0x29).
class ToFSensors {
public:
    static constexpr uint8_t ADDR_SENSOR1 = 0x30;  // reassigned from 0x29
    static constexpr uint8_t ADDR_SENSOR2 = 0x29;  // factory default

    ToFSensors(uint8_t xshut1, uint8_t xshut2);

    // Initializes both sensors. Returns false if either fails.
    bool begin();

    // Returns latest range in mm (continuous mode). 65535 on error/timeout.
    uint16_t readSensor1();
    uint16_t readSensor2();

private:
    uint8_t xshut1_;
    uint8_t xshut2_;
    VL53L0X sensor1_;
    VL53L0X sensor2_;
    bool ok_ = false;
};
