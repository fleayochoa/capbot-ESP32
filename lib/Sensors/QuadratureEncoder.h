#ifndef QUADRATURE_ENCODER_H
#define QUADRATURE_ENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h"

/**
 * Lector de encoder en cuadratura usando la API legacy de PCNT (driver/pcnt.h).
 *
 * Usa una unidad PCNT completa por encoder (ambos canales) para lograr
 * decodificación 4x: cuenta los 4 flancos (A↑, A↓, B↑, B↓) por ciclo.
 *
 * Mantiene un contador de 32 bits manejando el desbordamiento del
 * contador interno de 16 bits del PCNT vía interrupción de H_LIM/L_LIM.
 */
class QuadratureEncoder {
public:
    /**
     * @param unit    Unidad PCNT a usar (PCNT_UNIT_0 ... PCNT_UNIT_7)
     * @param pinA    GPIO de la señal A del encoder
     * @param pinB    GPIO de la señal B del encoder
     * @param filter  Valor del filtro antirrebote (0-1023, en ciclos APB 80 MHz).
     *                0 desactiva el filtro. 100 ≈ 1.25 µs.
     */
    QuadratureEncoder(pcnt_unit_t unit, gpio_num_t pinA, gpio_num_t pinB, uint16_t filter = 100);

    /** Inicializa el PCNT. Llamar una vez en setup(). */
    void begin();

    /** Devuelve la posición absoluta acumulada (32 bits con signo). */
    int32_t read();

    /** Pone el contador a cero. */
    void reset();

    /** Pausa el conteo (no se pierden pulsos, solo se detiene). */
    void pause();

    /** Reanuda el conteo. */
    void resume();

    /** Calcula los conteos por segundo. */
    float computeCountsPerSec();

private:
    pcnt_unit_t _unit;
    gpio_num_t  _pinA;
    gpio_num_t  _pinB;
    uint16_t    _filter;
    volatile int32_t _overflow;
    float _lastCountPerSec;
    unsigned long _lastTsUsForVel;
    int32_t _lastCountForVel;


    static QuadratureEncoder* _instances[PCNT_UNIT_MAX];
    static bool _isrServiceInstalled;


    static void IRAM_ATTR onOverflow(void* arg);
};

#endif // QUADRATURE_ENCODER_H