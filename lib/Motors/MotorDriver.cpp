#include "MotorDriver.h"

// ---- Inicialización de miembros ----
MotorDriver::MotorDriver(Capbot::motorPins leftMotorPins, Capbot::motorPins rightMotorPins
                        ,uint8_t leftCH, uint8_t rightCH) {
    left_.in1 = leftMotorPins.pinA;
    left_.in2 = leftMotorPins.pinB;
    left_.ena = leftMotorPins.ena;

    right_.in1 = rightMotorPins.pinA;
    right_.in2 = rightMotorPins.pinB;
    right_.ena = rightMotorPins.ena;

    Channel left_{
        left_.in1, left_.in2, left_.ena, leftCH};
    Channel right_{
        right_.in1, right_.in2, right_.ena, rightCH};
}

// ---- Helpers ----
static inline int16_t clamp16(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return static_cast<int16_t>(lo);
    if (v > hi) return static_cast<int16_t>(hi);
    return static_cast<int16_t>(v);
}

// ---- API pública ----
void MotorDriver::begin() {
    pinMode(left_.in1,  OUTPUT);
    pinMode(left_.in2,  OUTPUT);
    pinMode(right_.in1, OUTPUT);
    pinMode(right_.in2, OUTPUT);

    // LEDC: configurar cada canal con frecuencia + resolución, y ligarlo al
    // pin ENA. En arduino-esp32 v3 la API cambió a ledcAttach(pin, freq, res),
    // pero mantenemos la API clásica ledcSetup+ledcAttachPin para máxima
    // compatibilidad con versiones 2.x del core.
    ledcSetup(left_.ledcCh,  Cfg::PWM_FREQ_HZ, Cfg::PWM_RESOLUTION_BITS);
    ledcSetup(right_.ledcCh, Cfg::PWM_FREQ_HZ, Cfg::PWM_RESOLUTION_BITS);
    ledcAttachPin(left_.ena,  left_.ledcCh);
    ledcAttachPin(right_.ena, right_.ledcCh);

    // Arrancamos en freno activo: más seguro que coast (si el ESP32 arranca
    // antes que la Jetson, el robot no sale rodando por inercia).
    brake();
}

void MotorDriver::drive(int16_t left_cmd, int16_t right_cmd) {
    braking_ = false;
    applyChannel(left_,  left_cmd,  lastLeftPwm_);
    applyChannel(right_, right_cmd, lastRightPwm_);
}

void MotorDriver::brake() {
    braking_ = true;
    // IN1=IN2=0, ENA=max -> freno activo por corto a GND
    digitalWrite(left_.in1,  LOW);
    digitalWrite(left_.in2,  LOW);
    digitalWrite(right_.in1, LOW);
    digitalWrite(right_.in2, LOW);
    const uint32_t max_duty = (1u << Cfg::PWM_RESOLUTION_BITS) - 1u;
    ledcWrite(left_.ledcCh,  max_duty);
    ledcWrite(right_.ledcCh, max_duty);
    lastLeftPwm_  = 0;
    lastRightPwm_ = 0;
}

void MotorDriver::coast() {
    braking_ = false;
    digitalWrite(left_.in1,  LOW);
    digitalWrite(left_.in2,  LOW);
    digitalWrite(right_.in1, LOW);
    digitalWrite(right_.in2, LOW);
    ledcWrite(left_.ledcCh,  0);
    ledcWrite(right_.ledcCh, 0);
    lastLeftPwm_  = 0;
    lastRightPwm_ = 0;
}

// ---- Internals ----
void MotorDriver::applyChannel(const Channel& ch, int16_t cmd, int16_t& lastPwmOut) {
    if (cmd >= 0) {
        digitalWrite(ch.in1, HIGH);
        digitalWrite(ch.in2, LOW);
    } else {
        digitalWrite(ch.in1, LOW);
        digitalWrite(ch.in2, HIGH);
        cmd = static_cast<int16_t>(-cmd);  // magnitud para el PWM
    }
    const int16_t pwm = cmdToPwm(cmd);
    ledcWrite(ch.ledcCh, static_cast<uint32_t>(pwm));
    lastPwmOut = pwm;
}

int16_t MotorDriver::cmdToPwm(int16_t cmd) const {
    // cmd >= 0 aquí (applyChannel ya lo hizo positivo)
    const int32_t full = Cfg::CMD_FULL_SCALE;
    const int32_t max_pwm = (1 << Cfg::PWM_RESOLUTION_BITS) - 1;
    int32_t scaled = (static_cast<int32_t>(cmd) * max_pwm) / full;
    return clamp16(scaled, 0, max_pwm);
}