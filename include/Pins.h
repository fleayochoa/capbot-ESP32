// Mapa de pines — ajustar al hardware final.
//
// L298N: cada canal tiene IN1, IN2 y ENA (PWM).
//   IN1=HIGH, IN2=LOW  -> retrocede
//   IN1=LOW,  IN2=HIGH -> avanza
//   IN1=IN2 (ambos HIGH o ambos LOW) + ENA=HIGH -> FRENO ACTIVO (corto)
//   IN1=IN2 + ENA=LOW -> coast (libre)
//
// En ESP32 el PWM se hace con ledc (canales hardware), no analogWrite.

#pragma once
#include <Arduino.h>

namespace Pins {

// --- Motor izquierdo ---
// LEFT_IN1 está en GPIO4, no GPIO12: GPIO12 es un pin de strapping (MTDI,
// selecciona voltaje de flash al boot). El shield lo mantenía en HIGH
// durante el boot, lo que hacía que el ESP32 seleccionara flash a 1.8V en
// lugar de 3.3V y no llegara a correr el firmware (sin ESP_HELLO, sin nada).
constexpr uint8_t LEFT_IN1 = 12;
constexpr uint8_t LEFT_IN2 = 15;
constexpr uint8_t LEFT_ENA = 18;   // PWM

// --- Motor derecho ---
constexpr uint8_t RIGHT_IN1 = 14;
constexpr uint8_t RIGHT_IN2 = 13;
constexpr uint8_t RIGHT_ENA = 5;  // PWM

// --- Canales LEDC (ESP32 PWM) ---
constexpr uint8_t LEDC_CH_LEFT  = 0;
constexpr uint8_t LEDC_CH_RIGHT = 1;

// --- LED de estado (opcional) ---
constexpr uint8_t STATUS_LED = 2;  // LED integrado en ESP32 WROOM

// --- Reservados para sensores (encoders, I2C IMU, etc.)

// I2C compartido para la IMU MPU6050 (AD0=GND -> 0x68). Pines por defecto del
// ESP32 WROOM. La odometría on-board sólo lee el giroscopio (eje Z) de acá.
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;

constexpr uint8_t ENC_LEFT_A = 33;
constexpr uint8_t ENC_LEFT_B = 32;
constexpr uint8_t ENC_RIGHT_A = 26;
constexpr uint8_t ENC_RIGHT_B = 27;


}  // namespace Pins