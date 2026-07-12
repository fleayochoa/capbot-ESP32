/**
 * Lector del giroscopio de la IMU MPU6050 (familia MPU-60X0): acceso directo
 * por registros I2C, sin depender de una librería de terceros.
 *
 * Sólo expone la velocidad angular en el eje Z (yaw rate) — es lo único que
 * consume Odometry (fusión con el yaw derivado de los encoders). El
 * acelerómetro no se lee: no se usa para la estimación de velocidad/posición
 * (esa se calcula sólo con encoders, ver Odometry).
 *
 * No valida el registro WHO_AM_I contra un chip específico (sólo lo loguea):
 * variantes de la familia (MPU6050/MPU6500/...) comparten el mismo mapa de
 * registros básico aunque reporten IDs distintos, y un chequeo estricto sólo
 * generaría falsos "no responde".
 *
 * Dirección I2C 0x68 cuando AD0=GND, 0x69 cuando AD0=VCC. Comparte el bus I2C
 * con otros dispositivos. Sin dependencias externas más allá de Wire.
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include "Config.h"

class IMUSensor {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x68;  // AD0=GND

    explicit IMUSensor(uint8_t addr = DEFAULT_ADDR, TwoWire& wire = Wire);

    // Inicializa el chip (asume que Wire.begin() ya se llamó con los pines
    // correctos): reset, despierta, configura rango de gyro (±500 °/s) y
    // filtro digital (~10 Hz). Devuelve false sólo si el chip no responde en
    // el bus I2C (sin ACK); no valida el WHO_AM_I, sólo lo loguea.
    bool begin();

    // Ping I2C directo.
    bool isConnected();

    // Calibra el bias estático del giroscopio en Z con el robot QUIETO:
    // promedia `samples` lecturas (sólo las I2C exitosas). read() resta este
    // bias en cada lectura. Bloquea ~samples*delayMs; pensado para el
    // arranque (la Jetson tarda >10 s en levantar sus nodos). Devuelve false
    // si la IMU no está inicializada o si ninguna lectura fue exitosa.
    bool calibrate(uint16_t samples, uint16_t delayMs);

    bool  isCalibrated() const { return calibrated_; }
    float gyroZBias()    const { return gyroZBias_; }

    // Lee la velocidad angular en Z (yaw rate, rad/s), ya con el bias de
    // calibración restado (montaje derecho: X adelante, Z arriba, sin
    // remapeo de signo) y suavizada con filtro de mediana (IMU_MEDIAN_WINDOW)
    // seguido de media móvil (IMU_ROLLING_WINDOW). Devuelve false si la IMU
    // no se inicializó o si la lectura I2C falla; en ese caso deja gyroZ en
    // el último valor filtrado válido (no en cero) para no inyectar un salto
    // artificial en los buffers de mediana/media móvil ni en la odometría.
    bool read(float& gyroZ);

    bool ok() const { return ok_; }

private:
    // ---- Registros (comunes a MPU6050/MPU6500) ----
    static constexpr uint8_t REG_SMPLRT_DIV  = 0x19;
    static constexpr uint8_t REG_CONFIG      = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;  // burst de 2 bytes: sólo Z
    static constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I    = 0x75;

    // Escala para FS_SEL=1 (±500 °/s), configurado en begin().
    static constexpr float GYRO_LSB_PER_DPS = 65.5f;

    void    writeReg8(uint8_t reg, uint8_t value);
    uint8_t readReg8(uint8_t reg);
    bool    readBurst(uint8_t reg, uint8_t* buf, uint8_t len);

    // Lee gyro.z escalado a rad/s en el marco crudo del sensor: sin resta de
    // bias ni remapeo. calibrate() y read() comparten esta base.
    bool readRawGyroZ(float& gz);

    // Empuja una muestra a la ventana deslizante y sobrescribe gz con el
    // promedio de la ventana.
    void rollingMean(float& gz);

    // Empuja una muestra a la ventana de mediana y sobrescribe gz con la
    // mediana de la ventana (promedio de los dos valores centrales si el
    // tamaño de ventana efectivo es par). Corre antes de rollingMean().
    void medianFilter(float& gz);

    uint8_t  addr_;
    TwoWire* wire_;
    bool     ok_ = false;

    float gyroZBias_  = 0.0f;
    bool  calibrated_ = false;

    // Último valor filtrado válido: se devuelve en vez de un 0 artificial
    // cuando una lectura I2C falla, para no contaminar los filtros ni la
    // odometría con un salto falso.
    float lastGyroZ_ = 0.0f;

    static constexpr uint8_t ROLL_WIN = Cfg::IMU_ROLLING_WINDOW;
    float   gzBuf_[ROLL_WIN]{};
    uint8_t rollIdx_  = 0;
    bool    rollFull_ = false;

    static constexpr uint8_t MEDIAN_WIN = Cfg::IMU_MEDIAN_WINDOW;
    float   medBuf_[MEDIAN_WIN]{};
    uint8_t medIdx_  = 0;
    bool    medFull_ = false;
};
