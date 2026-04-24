#include "QuadratureEncoder.h"

QuadratureEncoder* QuadratureEncoder::_instances[PCNT_UNIT_MAX] = {nullptr};
bool QuadratureEncoder::_isrServiceInstalled = false;

QuadratureEncoder::QuadratureEncoder(pcnt_unit_t unit, gpio_num_t pinA, gpio_num_t pinB, uint16_t filter )
    : _unit(unit), _pinA(pinA), _pinB(pinB), _filter(filter), _overflow(0), _lastCountPerSec(0.0f)
    , _lastTsUsForVel(0), _lastCountForVel(0) {
}

void IRAM_ATTR QuadratureEncoder::onOverflow(void* arg) {
    pcnt_unit_t unit = (pcnt_unit_t)(uint32_t)arg;
    QuadratureEncoder* enc = _instances[unit];
    if (!enc) return;

    uint32_t status = 0;
    pcnt_get_event_status(unit, &status);

    if (status & PCNT_EVT_H_LIM) {
        enc->_overflow += INT16_MAX;
    }
    if (status & PCNT_EVT_L_LIM) {
        enc->_overflow += INT16_MIN;
    }
}

void QuadratureEncoder::begin() {
    _instances[_unit] = this;

    // Canal 0: pulso = A, control = B
    //   A↑ con B=low  -> INC    (avance)
    //   A↓ con B=high -> INC    (avance)
    //   A↑ con B=high -> DEC    (retroceso)
    //   A↓ con B=low  -> DEC    (retroceso)
    pcnt_config_t cfg0 = {};
    cfg0.pulse_gpio_num = _pinA;
    cfg0.ctrl_gpio_num  = _pinB;
    cfg0.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg0.hctrl_mode     = PCNT_MODE_KEEP;
    cfg0.pos_mode       = PCNT_COUNT_DEC;   // A↑
    cfg0.neg_mode       = PCNT_COUNT_INC;   // A↓
    cfg0.counter_h_lim  = INT16_MAX;
    cfg0.counter_l_lim  = INT16_MIN;
    cfg0.unit           = _unit;
    cfg0.channel        = PCNT_CHANNEL_0;
    pcnt_unit_config(&cfg0);

    // Canal 1: pulso = B, control = A
    //   B↑ con A=high -> INC    (avance)
    //   B↓ con A=low  -> INC    (avance)
    //   B↑ con A=low  -> DEC    (retroceso)
    //   B↓ con A=high -> DEC    (retroceso)
    pcnt_config_t cfg1 = {};
    cfg1.pulse_gpio_num = _pinB;
    cfg1.ctrl_gpio_num  = _pinA;
    cfg1.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg1.hctrl_mode     = PCNT_MODE_KEEP;
    cfg1.pos_mode       = PCNT_COUNT_INC;   // B↑
    cfg1.neg_mode       = PCNT_COUNT_DEC;   // B↓
    cfg1.counter_h_lim  = INT16_MAX;
    cfg1.counter_l_lim  = INT16_MIN;
    cfg1.unit           = _unit;
    cfg1.channel        = PCNT_CHANNEL_1;
    pcnt_unit_config(&cfg1);

    // Filtro anti-glitch (filtra pulsos menores al valor dado en ciclos APB)
    if (_filter > 0) {
        pcnt_set_filter_value(_unit, _filter);
        pcnt_filter_enable(_unit);
    } else {
        pcnt_filter_disable(_unit);
    }

    // Habilitar eventos de overflow/underflow
    pcnt_event_enable(_unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(_unit, PCNT_EVT_L_LIM);

    // Instalar servicio ISR una sola vez para todas las unidades
    if (!_isrServiceInstalled) {
        pcnt_isr_service_install(0);
        _isrServiceInstalled = true;
    }
    pcnt_isr_handler_add(_unit, onOverflow, (void*)_unit);

    // Arrancar contador
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    _overflow = 0;
    pcnt_counter_resume(_unit);
}

int32_t QuadratureEncoder::read() {
    int16_t count = 0;
    // Leer contador y overflow de forma coherente: deshabilitar interrupciones
    // brevemente para evitar que un overflow ocurra entre ambas lecturas.
    portDISABLE_INTERRUPTS();
    pcnt_get_counter_value(_unit, &count);
    int32_t total = _overflow + count;
    portENABLE_INTERRUPTS();
    return total;
}

void QuadratureEncoder::reset() {
    pcnt_counter_pause(_unit);
    pcnt_counter_clear(_unit);
    _overflow = 0;
    pcnt_counter_resume(_unit);
}

void QuadratureEncoder::pause() {
    pcnt_counter_pause(_unit);
}

void QuadratureEncoder::resume() {
    pcnt_counter_resume(_unit);
}

float QuadratureEncoder::computeCountsPerSec() {
    const int32_t  cur = this->read();
    const uint32_t now = micros();
 
    if (_lastTsUsForVel == 0) {
        // Primera llamada: no hay base para estimar velocidad
        _lastCountForVel = cur;
        _lastTsUsForVel  = now;
        return 0.0f;
    }
 
    const uint32_t dt_us = now - _lastTsUsForVel;
    if (dt_us < 1000) {
        return _lastCountPerSec;  // <1ms, demasiado ruidoso para derivar
    }
 
    const int32_t dcount = cur - _lastCountForVel;
    const float cps = (static_cast<float>(dcount) * 1000000.0f) / static_cast<float>(dt_us);
 
    _lastCountForVel = cur;
    _lastTsUsForVel  = now;
    _lastCountPerSec = cps;
    return cps;
}