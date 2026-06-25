/**
 * Lector del sensor IMU (GY-MPU6050) usando la librería Adafruit_MPU6050.
 *
 * Dirección I2C por defecto 0x68 (AD0 a GND), 0x69 cuando AD0 está a VCC.
 * Solo se exponen acelerómetro y giroscopio: el MPU6050 no tiene magnetómetro
 * ni fusión de sensores interna (a diferencia del BNO055 que reemplaza).
 * El IMU se conecta al bus I2C compartido con otros dispositivos como los TOF.
 *
 * Dependencias (agregar a platformio.ini):
 * lib_deps =
 *      ...
 *      adafruit/Adafruit MPU6050 @ ^2.2.6
 *      adafruit/Adafruit Unified Sensor @ ^1.1.15
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include <Adafruit_MPU6050.h>

class IMUSensor {
public:

    static constexpr uint8_t DEFAULT_ADDR = 0x68;

    struct Vec3 { float x, y, z; };

    explicit IMUSensor(uint8_t addr = DEFAULT_ADDR, TwoWire& wire = Wire);

    // Inicializa I2C y verifica el chip.
    bool begin();

    // Ping I2C directo. No usa la lib de Adafruit (no expone este check).
    bool isConnected();

    // Calibración estática: el robot debe quedar quieto. Promedia `samples`
    // lecturas crudas y guarda el resultado como error sistemático (bias) de
    // cada eje, que se resta de las lecturas posteriores.
    void calibrate(uint16_t samples = 1000);

    // ---- Lecturas (ya corregidas por el bias de calibrate()) ----
    Vec3 readAccel();   // m/s²
    Vec3 readGyro();    // dps

private:
    uint8_t          addr_;
    TwoWire*         wire_;
    Adafruit_MPU6050 mpu_;
    Vec3             accelBias_{};
    Vec3             gyroBias_{};
};
