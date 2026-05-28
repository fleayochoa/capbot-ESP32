// Controlador PID estándar para la regulación de lazos de control en el ESP32.
//
// Implementa el algoritmo posicional clásico con:
//   - Ganancias configurables (Kp, Ki, Kd)
//   - Saturación (clamping) de la acción integral (anti-windup)
//   - Saturación de la salida final
//
// El tiempo de muestreo (dt) se pasa de forma explícita en cada llamada a compute()
// para desacoplar el cálculo del temporizador global, facilitando su uso en tareas
// periódicas de FreeRTOS o temporizadores por hardware.

#pragma once
#include <Arduino.h>
#include <stdint.h>

class PidController {
public:
    struct Config {
        float kp;
        float ki;
        float kd;
        float outMin;
        float outMax;
        float maxIntegral; // Límite absoluto para evitar windup excesivo
    };

    explicit PidController(const Config& config);

    // Reinicia el estado interno (término integral y error previo).
    // Se debe llamar al habilitar o reanudar el bucle de control.
    void reset();

    // Actualiza las ganancias y límites en tiempo de ejecución.
    void setConfig(const Config& config);

    // Ejecuta una iteración del bucle PID.
    //   setpoint    : Valor deseado (referencia)
    //   measurement : Valor actual medido (realimentación)
    //   dt          : Tiempo transcurrido desde la última ejecución (en segundos)
    // Retorna la señal de control calculada y saturada.
    float compute(float error, float dt);

    // Getters para inspección o telemetría
    float lastOutput() const { return lastOutput_; }
    // float integralError() const { return integralError_; }
    
    const Config& config() const { return config_; }

private:
    Config config_;

    // float integralError_ = 0.0f;
    // float lastError_     = 0.0f;
    float errorBuf_[3] = {0.0f, 0.0f, 0.0f}; // Buffer circular para error en t, t-1 y t-2
    float lastOutput_    = 0.0f;
};