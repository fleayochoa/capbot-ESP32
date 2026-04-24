// Driver L298N para dos motores DC con PWM por hardware (LEDC).
//
// Cada canal usa 3 pines:
//   IN1, IN2 : dirección (GPIO digital)
//   ENA      : PWM (canal LEDC)
//
// Tabla de verdad L298N (por canal):
//   IN1=1, IN2=0, ENA=PWM  -> avanza con duty=PWM
//   IN1=0, IN2=1, ENA=PWM  -> retrocede con duty=PWM
//   IN1=1, IN2=1, ENA=1    -> FRENO ACTIVO (corto a VCC)
//   IN1=0, IN2=0, ENA=1    -> FRENO ACTIVO (corto a GND, recomendado)
//   IN1=x, IN2=x, ENA=0    -> coast / libre
//
// Usamos IN1=IN2=0 + ENA=HIGH como freno activo (más habitual).
//
// El comando entra en unidades del host (int16 en rango [-CMD_FULL_SCALE,
// +CMD_FULL_SCALE]) y se mapea internamente al duty PWM.

#pragma once
#include <Arduino.h>
#include <stdint.h>

#include "Config.h"
#include "Pins.h"
#include "CapTypes.h"

class MotorDriver {
public:
    MotorDriver(Capbot::motorPins leftMotorPins, Capbot::motorPins rightMotorPins
                , uint8_t leftCH, uint8_t rightCH);
    // Configura pines y canales LEDC. Llamar en setup().
    void begin();

    // Comando por rueda en unidades del host (-CMD_FULL_SCALE..+CMD_FULL_SCALE).
    // Valores fuera de rango se saturan.
    void drive(int16_t left_cmd, int16_t right_cmd);

    // Freno activo en ambos motores (cortocircuita los terminales a GND).
    // Se mantiene hasta el siguiente drive() o coast().
    void brake();

    // Libera los motores (coast). No frena: la inercia los sigue moviendo.
    void coast();

    // Último duty aplicado, útil para telemetría (-1023..1023)
    int16_t leftPwm() const { return lastLeftPwm_; }
    int16_t rightPwm() const { return lastRightPwm_; }
    bool isBraking() const { return braking_; }

private:
    struct Channel {
        uint8_t in1;
        uint8_t in2;
        uint8_t ena;
        uint8_t ledcCh;
    };

    // Aplica un comando a UN canal. cmd en rango host.
    void applyChannel(const Channel& ch, int16_t cmd, int16_t& lastPwmOut);

    // Convierte comando host [-CMD_FULL_SCALE..+CMD_FULL_SCALE] a duty LEDC
    // con resolución PWM_RESOLUTION_BITS. Clamp incluido.
    int16_t cmdToPwm(int16_t cmd) const;

    Channel left_{
        0, 0, 0, 0};  // se inicializan en el constructor
    Channel right_{
        0, 0, 0, 0};  // se inicializan en el constructor

    int16_t lastLeftPwm_  = 0;
    int16_t lastRightPwm_ = 0;
    bool    braking_      = false;
};