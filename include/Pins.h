// Mapa de pines — ajustar al hardware final.
//
// L298N: cada canal tiene IN1, IN2 y ENA (PWM).
//   IN1=HIGH, IN2=LOW  -> avanza
//   IN1=LOW,  IN2=HIGH -> retrocede
//   IN1=IN2 (ambos HIGH o ambos LOW) + ENA=HIGH -> FRENO ACTIVO (corto)
//   IN1=IN2 + ENA=LOW -> coast (libre)
//
// En ESP32 el PWM se hace con ledc (canales hardware), no analogWrite.

#pragma once
#include <Arduino.h>

namespace Pins {

// --- Motor izquierdo ---
constexpr uint8_t LEFT_IN1 = 25;
constexpr uint8_t LEFT_IN2 = 26;
constexpr uint8_t LEFT_ENA = 27;   // PWM

// --- Motor derecho ---
constexpr uint8_t RIGHT_IN1 = 32;
constexpr uint8_t RIGHT_IN2 = 33;
constexpr uint8_t RIGHT_ENA = 14;  // PWM

// --- Canales LEDC (ESP32 PWM) ---
constexpr uint8_t LEDC_CH_LEFT  = 0;
constexpr uint8_t LEDC_CH_RIGHT = 1;

// --- LED de estado (opcional) ---
constexpr uint8_t STATUS_LED = 2;  // LED integrado en muchos dev-boards

// --- Reservados para sensores (I2C IMU, encoders, etc.) ---
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t ENC_LEFT_A = 33;  // input-only en ESP32, ok para encoder
constexpr uint8_t ENC_LEFT_B = 32;
constexpr uint8_t ENC_RIGHT_A = 26;
constexpr uint8_t ENC_RIGHT_B = 27;

}  // namespace Pins