/**
 * Lector del sensor IMU (GY-BNO055) usando la librería Adafruit_BNO055.
 * 
 * Dirección I2C por defecto 0x28 cuando ADC está cortocircuitado a GND, 0x29 cuando está a VCC.
 * Se usa la librería Adafruit_BNO055, que maneja la configuración y lectura del sensor. 
 * El IMU se conecta al bus I2C compartido con otros dispositivos como los TOF.
 * 
 * Dependencias (agregar a platformio.ini):
 * lib_deps =
 *      ...
 *      adafruit/Adafruit BNO055 @ ^1.6.4
 *       adafruit/Adafruit Unified Sensor @ ^1.1.14
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMUSensor {
public:
    
    static constexpr uint8_t DEFAULT_ADDR = 0x28;
 
    // Modos relevantes.
    enum class OpMode : uint8_t {
        CONFIG   = OPERATION_MODE_CONFIG,
        ACCONLY  = OPERATION_MODE_ACCONLY,
        MAGONLY  = OPERATION_MODE_MAGONLY,
        GYROONLY = OPERATION_MODE_GYRONLY,
        AMG      = OPERATION_MODE_AMG,
        IMU      = OPERATION_MODE_IMUPLUS,   // fusión accel+gyro
        COMPASS  = OPERATION_MODE_COMPASS,
        M4G      = OPERATION_MODE_M4G,
        NDOF_FMC_OFF = OPERATION_MODE_NDOF_FMC_OFF,
        NDOF     = OPERATION_MODE_NDOF,      // fusión 9DOF completa
    };
 
    struct Vec3   { float x, y, z; };
    struct Euler  { float heading, roll, pitch; };   // grados
    struct Quat   { float w, x, y, z; };
    struct CalibStatus { uint8_t sys, gyr, acc, mag; };  // 0..3 c/u
 
    explicit IMUSensor(uint8_t addr = DEFAULT_ADDR, TwoWire& wire = Wire);
 
    // Inicializa I2C, verifica el chip y configura el modo. useExternalCrystal
    // activa el XTAL de 32.768 kHz del GY-BNO055 (recomendado).
    bool begin(OpMode mode = OpMode::NDOF, bool useExternalCrystal = true);
 
    // Ping I2C directo. No usa la lib de Adafruit (no expone este check).
    bool isConnected();
 
    bool setMode(OpMode m);
    OpMode mode() const { return mode_; }
 
    // ---- Lecturas ----
    // Adafruit devuelve Vector<3> sin indicar error I2C. Aquí siempre retornan
    // true; si hubo fallo de bus, los valores serán 0/inconsistentes. Para
    // detectar desconexión usar isConnected() periódicamente.
    Vec3 readAccel       ();   // m/s²
    Vec3 readGyro        ();   // dps (convertido desde rps interno)
    Vec3 readMag         ();   // µT
    bool readEuler       (Euler& e);   // deg
    bool readQuaternion  (Quat&  q);
    bool readLinearAccel (Vec3&  v);   // m/s² (sólo modos fusión)
    bool readGravity     (Vec3&  v);   // m/s² (sólo modos fusión)
    bool readCalibStatus (CalibStatus& c);
    bool readTemperature (int8_t& t_c);
 
    // Helper: true si los 4 subsistemas están calibrados (sys/gyr/acc/mag == 3).
    bool isFullyCalibrated();
 
    // Acceso al objeto interno por si el consumidor necesita APIs avanzadas
    // de Adafruit (offsets, self-test, event-based reads, etc.).
    Adafruit_BNO055& raw() { return bno_; }
 
private:
    uint8_t          addr_;
    TwoWire*         wire_;
    Adafruit_BNO055  bno_;
    OpMode           mode_ = OpMode::CONFIG;
 
    // Helper: copia imu::Vector<3> a Vec3 con un escalar opcional.
    static void copyVec(const imu::Vector<3>& src, Vec3& dst, float scale = 1.0f);
};
