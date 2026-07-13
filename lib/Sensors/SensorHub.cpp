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
    // 100 kHz (Standard Mode): de sobra para un burst de 2 bytes a la
    // cadencia de sampleImu() (ver Cfg::IMU_SAMPLE_PERIOD_MS), y evita
    // arriesgar el margen del datasheet (Fast Mode, 400 kHz máx).
    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
    Wire.setClock(100000);
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
    // I/O (I2C/PCNT) fuera del lock: portENTER_CRITICAL en ESP32 desactiva
    // interrupciones en este núcleo, así que la sección crítica debe ser lo
    // más corta posible (sólo las asignaciones al struct compartido).
    const int32_t encLeft   = encL_.read();
    const int32_t encRight  = encR_.read();
    const float   velLeft   = encL_.computeCountsPerSec();
    const float   velRight  = encR_.computeCountsPerSec();
    const uint32_t uptime   = millis();

    portENTER_CRITICAL(&mux_);
    last_.enc_left      = encLeft;
    last_.enc_right     = encRight;
    last_.vel_left_cps  = velLeft;
    last_.vel_right_cps = velRight;
    last_.uptime_ms     = uptime;
    portEXIT_CRITICAL(&mux_);
}

void SensorHub::sampleImu() {
    // Si no está viva, reintenta reconectar (con cooldown propio, ver
    // IMUSensor::tryReconnect) en vez de leer a ciegas para siempre.
    imu_.tryReconnect();

    // Si la IMU falló al iniciar, read() deja gyro_z en cero y la odometría
    // degrada theta/omega a sólo-encoders sin inyectar basura.
    float gyroZ;
    imu_.read(gyroZ);
    const bool alive = imu_.isAlive();

    portENTER_CRITICAL(&mux_);
    // Vehículo confirmado quieto (encoders y PWM en 0 en ambas ruedas, según
    // la última sample()/feedMotorStatus()): se descarta la lectura del
    // giroscopio de este ciclo y se fuerza a 0. Con ambas ruedas sin girar y
    // sin esfuerzo comandado, el yaw real es 0; sin esto, ruido/drift
    // térmico del gyro se integraría en Odometry como un yaw falso incluso
    // con el robot parado.
    const bool stationary = last_.vel_left_cps  == 0.0f && last_.vel_right_cps == 0.0f &&
                            last_.motor_pwm_left == 0    && last_.motor_pwm_right == 0;
    last_.imu_gyro_z = stationary ? 0.0f : gyroZ;
    last_.imu_alive  = alive;
    portEXIT_CRITICAL(&mux_);
}

void SensorHub::feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking) {
    portENTER_CRITICAL(&mux_);
    last_.motor_pwm_left  = leftPwm;
    last_.motor_pwm_right = rightPwm;
    last_.braking         = braking;
    portEXIT_CRITICAL(&mux_);
}

SensorHub::Telemetry SensorHub::last() const {
    portENTER_CRITICAL(&mux_);
    const Telemetry copy = last_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap, const StateEstimate& state,
                               const char* mode, const ControlTelemetry& ctrl) {
    StaticJsonDocument<768> doc;

    // imu_alive vía last() (no last_ directo): lo escribe ImuTaskCode, esta
    // función corre en TelemetryTaskCode -- misma sección crítica que
    // protege al lector cross-core en ControlTaskCode.
    portENTER_CRITICAL(&mux_);
    const bool imuAlive = last_.imu_alive;
    portEXIT_CRITICAL(&mux_);

    doc["mode"] = mode;
    doc["imu_alive"] = imuAlive;  // ver IMUSensor::isAlive()

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