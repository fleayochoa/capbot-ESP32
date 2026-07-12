#include "SensorHub.h"
#include "Odometry.h"
#include "Pins.h"

#include <Wire.h>
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

    // Bus I2C compartido para la IMU. Se inicializa acá (no en IMUSensor) para
    // fijar los pines del ESP32 y la velocidad del bus una sola vez.
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(400000);
    if (!imu_.begin()) {
        Serial.println("WARN: IMU no responde; odometría (theta/omega) sólo con encoders");
    } else {
        // Calibración automática con el robot quieto. Bloquea unos segundos,
        // pero corre en setup() mucho antes de que la Jetson mande comandos.
        Serial.println("Calibrando IMU: mantener el robot QUIETO...");
        imu_.calibrate(Cfg::IMU_CALIB_SAMPLES, Cfg::IMU_CALIB_DELAY_MS);
        Serial.printf("IMU calibrada: gyroZBias=%.4f rad/s\n", imu_.gyroZBias());
    }

    Serial.println("SensorHub initialized");
}

void SensorHub::sample() {
    last_.enc_left         = encL_.read();
    last_.enc_right        = encR_.read();
    last_.vel_left_cps     = encL_.computeCountsPerSec();
    last_.vel_right_cps    = encR_.computeCountsPerSec();
    // Si la IMU falló al iniciar, read() deja gyro_z en cero y la odometría
    // degrada theta/omega a sólo-encoders sin inyectar basura.
    imu_.read(last_.imu_gyro_z);
    last_.uptime_ms        = millis();
}

void SensorHub::feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking) {
    last_.motor_pwm_left  = leftPwm;
    last_.motor_pwm_right = rightPwm;
    last_.braking         = braking;
}

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap, const StateEstimate& state,
                               const char* mode, const ControlTelemetry& ctrl) {
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

    // Odometría on-board (encoders + IMU). Ángulos en grados para mantener el
    // contrato previo con la Jetson (a = heading, w = vel. angular).
    JsonObject odo = doc.createNestedObject("odo");
        odo["x"] = state.x;                       // m
        odo["y"] = state.y;                       // m
        odo["a"] = state.theta * RAD_TO_DEG;      // grados
        odo["v"] = state.v;                       // m/s
        odo["w"] = state.omega * RAD_TO_DEG;      // grados/s

    JsonObject c = doc.createNestedObject("ctrl");
        c["sp_left"]  = ctrl.sp_left;
        c["sp_right"] = ctrl.sp_right;

    const size_t n = serializeJson(doc, out, out_cap);
    if (n == 0 || n >= out_cap) return 0;
    return n;
}