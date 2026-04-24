#pragma once  // Obligatorio: Evita que el archivo se incluya múltiples veces y cause errores
#include <stdint.h>
#include <Arduino.h>

namespace Capbot{
    struct motorPins{
        uint8_t pinA;
        uint8_t pinB;
        uint8_t ena;
    };
    struct encoderPins{
        gpio_num_t leftA;
        gpio_num_t leftB;
        gpio_num_t rightA;
        gpio_num_t rightB;
    };
}