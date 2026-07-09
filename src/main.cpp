// main.cpp — Entry point del firmware.
//
// Modos (ver Cfg::MsgType::MODE_CMD):
//   0 MANUAL         : MOTOR_CMD de teleop (PWM crudo) maneja los motores
//                      directo.
//   1 AUTONOMOUS_NAV : VEL_CMD lleva el setpoint de velocidad por rueda,
//                      calculado en la Jetson (jetson-bridge) a partir de
//                      /cmd_vel + cinemática diferencial: float32 wheelLeft
//                      (rad/s), float32 wheelRight (rad/s). El PID de
//                      velocidad on-board (Controlador::computeVelocity)
//                      corre un lazo independiente por rueda contra el
//                      encoder (sin cinemática ni odometría en el ESP32) y
//                      maneja los motores. Es el único modo autónomo: la
//                      navegación/pose vive en nav2 + EKF, en la Jetson.
//                      MOTOR_CMD se ignora en este modo.
//
// setup():
//   1. Inicializa Serial con la Jetson
//   2. Inicializa motores (arrancan en freno por seguridad)
//   3. Inicializa sensores (encoders) y odometría
//   4. Registra callbacks en JetsonLink
//   5. Manda ESP_HELLO
//
// loop():
//   1. JetsonLink.tick()  -> procesa RX y dispara callbacks
//   2. Watchdog: si !hb y lastRx > JETSON_WATCHDOG_MS -> brake() y sale a manual
//   3. Cada TELEMETRY_PERIOD_MS: sample + odometría + control + sendTelemetry
//   4. Pequeño yield para no hambrear tareas RTOS de fondo

#include <Arduino.h>

#include "Config.h"
#include "Pins.h"
#include "JetsonLink.h"
#include "MotorDriver.h"
#include "SensorHub.h"
#include "Control.h"
#include "CapTypes.h"

// ---- Types ----
Capbot::encoderPins encPins = {
    static_cast<gpio_num_t>(Pins::ENC_LEFT_A), static_cast<gpio_num_t>(Pins::ENC_LEFT_B),
    static_cast<gpio_num_t>(Pins::ENC_RIGHT_A), static_cast<gpio_num_t>(Pins::ENC_RIGHT_B)
};
Capbot::motorPins leftMotorPins  = {Pins::LEFT_IN1,  Pins::LEFT_IN2,  Pins::LEFT_ENA};
Capbot::motorPins rightMotorPins = {Pins::RIGHT_IN1, Pins::RIGHT_IN2, Pins::RIGHT_ENA};

// ---- Default controller config (gains tunable at runtime via PID_PARAM) ----
//   leftWheelPid / rightWheelPid : rad/s -> esfuerzo motor [-32767, 32767]
// PLACEHOLDER: ganancias estimadas al convertir desde el PID lineal
// anterior (m/s) por el radio de rueda (~0.0335 m). Falta retunear en
// hardware ahora que el error de entrada es rad/s por rueda.
static const Controlador::Config DEFAULT_CTRL_CFG = {
    { 0.15f, 0.01f, 0.0f, -32767.0f , 32767.0f , 50000.0f },  // leftWheelPid
    { 0.15f, 0.01f, 0.0f, -32767.0f , 32767.0f , 50000.0f },  // rightWheelPid
    32767.0f    // maxMotorOutput
};

// ---- Instancias globales ----
static JetsonLink   g_link;
static MotorDriver  g_motors(leftMotorPins, rightMotorPins, Pins::LEDC_CH_LEFT, Pins::LEDC_CH_RIGHT);
static SensorHub    g_sensors(encPins, PCNT_UNIT_0, PCNT_UNIT_1, 100);
static Controlador  g_controller(DEFAULT_CTRL_CFG);

// Cuentas/seg -> rad/s de rueda (decodificación 4x, ver Cfg::WHEEL_CPR).
static constexpr float CPS_TO_RADPS = (2.0f * PI) / Cfg::WHEEL_CPR;

// Último target de velocidad por rueda enviado a los motores (telemetría).
static struct {
    float left  = 0.0f;  // rad/s
    float right = 0.0f;  // rad/s
} g_setpoint;

// Setpoint de velocidad por rueda recibido vía VEL_CMD en AUTONOMOUS_NAV.
static struct {
    float    left   = 0.0f;  // rad/s
    float    right  = 0.0f;  // rad/s
    uint32_t lastMs = 0;     // millis() del último VEL_CMD recibido en este modo
} g_wheelSp;

enum class RobotMode : uint8_t {
    MANUAL         = 0,  // Teleop: MOTOR_CMD (PWM crudo) maneja directo
    AUTONOMOUS_NAV = 1,  // PID de velocidad on-board sobre setpoint de nav2 (VEL_CMD)
};

static RobotMode g_mode             = RobotMode::MANUAL;
static uint32_t  g_lastTelemetryMs  = 0;
static bool      g_watchdogTriggered = false;


// Doble nucleo
TaskHandle_t ControlTaskHandle;
TaskHandle_t TelemetryTaskHandle;
// ==============================================================
// Callbacks del JetsonLink
// ==============================================================
static void onMotorCmd(int16_t left, int16_t right, int16_t aux, void* /*ctx*/) {
    (void)aux;
    if (g_mode == RobotMode::MANUAL) {
        // Teleop: left/right son PWM crudo, manejan los motores directo.
        g_motors.drive(left, right);
    }
    // En AUTONOMOUS_NAV se ignora: el setpoint de velocidad por rueda llega
    // por VEL_CMD (onWheelVelCmd) y el PID on-board es la única autoridad
    // sobre los motores.
    g_watchdogTriggered = false;
}

static void onWheelVelCmd(float wheelLeft, float wheelRight, void* /*ctx*/) {
    if (g_mode == RobotMode::AUTONOMOUS_NAV) {
        // wheelLeft/wheelRight: rad/s, setpoint por rueda ya calculado por
        // la Jetson (cinemática diferencial sobre /cmd_vel). Sólo guardamos
        // el setpoint; runVelocityControl() en ControlTaskCode corre el PID
        // por rueda a dt fijo (20ms) contra el encoder.
        g_wheelSp.left  = wheelLeft;
        g_wheelSp.right = wheelRight;
        g_wheelSp.lastMs = millis();
    }
    // Fuera de AUTONOMOUS_NAV se ignora: el teleop crudo (MANUAL) es la
    // única autoridad sobre los motores en ese modo.
    g_watchdogTriggered = false;
}

static void onBrake(void* /*ctx*/) {
    // Vuelve a MANUAL: si no, un MOTOR_CMD viejo en cola seguiría manejando
    // los motores y pisaría este freno en el siguiente ciclo.
    g_mode = RobotMode::MANUAL;
    g_controller.reset();
    g_motors.brake();
}

static void onHeartbeat(void* /*ctx*/) {
    g_watchdogTriggered = false;
}

// ctrl_id: 0=leftWheelPid, 1=rightWheelPid (ver protocol/udp_frame.py CTRL_*).
static void onPidParam(uint8_t ctrl_id, uint8_t param_id, float value, void* /*ctx*/) {
    Controlador::Config cfg = g_controller.config();
    PidController::Config* pid = nullptr;
    switch (ctrl_id) {
        case 0: pid = &cfg.leftWheelPid;   break;
        case 1: pid = &cfg.rightWheelPid;  break;
        default: return;
    }
    switch (param_id) {
        case 0: pid->kp = value; break;
        case 1: pid->ki = value; break;
        case 2: pid->kd = value; break;
        default: return;
    }
    g_controller.setConfig(cfg);
}

static void onModeCmd(uint8_t mode, void* /*ctx*/) {
    switch (mode) {
        case 1:  g_mode = RobotMode::AUTONOMOUS_NAV; break;
        default: g_mode = RobotMode::MANUAL;         break;
    }
    // Limpia integradores del PID ante cualquier cambio de modo para que no
    // arrastre estado viejo si se vuelve a entrar a NAV.
    g_controller.reset();
    if (g_mode == RobotMode::MANUAL) {
        g_motors.brake();
    } else if (g_mode == RobotMode::AUTONOMOUS_NAV) {
        // Sin esto, un VEL_CMD viejo de una sesión NAV anterior se vería
        // como "setpoint fresco" y el robot arrancaría a moverse solo al
        // entrar al modo, antes de que la Jetson mande nada nuevo.
        g_wheelSp.left  = 0.0f;
        g_wheelSp.right = 0.0f;
        g_wheelSp.lastMs = 0;
    }
}

// ==============================================================
// Tareas del loop
// ==============================================================
static void runWatchdog() {
    const uint32_t last = g_link.lastRxMs();
    if (last == 0) {
        if (!g_motors.isBraking()) g_motors.brake();
        return;
    }
    const uint32_t since = millis() - last;
    if (since > Cfg::JETSON_WATCHDOG_MS) {
        if (!g_watchdogTriggered) {
            // Igual que onBrake(): sin esto, ControlTaskCode sigue corriendo
            // en AUTONOMOUS_NAV con setpoints viejos y pisa el freno en <20ms.
            g_mode = RobotMode::MANUAL;
            g_controller.reset();
            g_motors.brake();
            g_watchdogTriggered = true;
        }
    }
}

static void runVelocityControl(float dt) {
    if (millis() - g_wheelSp.lastMs > Cfg::NAV_VEL_TIMEOUT_MS) {
        // La Jetson dejó de mandar VEL_CMD: frenamos pero nos quedamos en
        // AUTONOMOUS_NAV a la espera de un setpoint fresco.
        if (!g_motors.isBraking()) g_motors.brake();
        return;
    }

    Controlador::State st;
    st.leftWheelVel  = g_sensors.last().vel_left_cps  * CPS_TO_RADPS;
    st.rightWheelVel = g_sensors.last().vel_right_cps * CPS_TO_RADPS;

    Controlador::WheelSetpoint sp;
    sp.leftWheelVel  = g_wheelSp.left;
    sp.rightWheelVel = g_wheelSp.right;

    const Controlador::MotorCommand cmd = g_controller.computeVelocity(sp, st, dt);
    g_setpoint.left  = sp.leftWheelVel;
    g_setpoint.right = sp.rightWheelVel;
    g_motors.drive(static_cast<int16_t>(cmd.left), static_cast<int16_t>(cmd.right));
}

static void runTelemetry() {
    const uint32_t now = millis();
    if (now - g_lastTelemetryMs < Cfg::TELEMETRY_PERIOD_MS) return;
    g_lastTelemetryMs = now;

    g_sensors.sample();
    g_sensors.feedMotorStatus(g_motors.leftPwm(), g_motors.rightPwm(), g_motors.isBraking());

    SensorHub::ControlTelemetry ctrl;
    ctrl.sp_left  = g_setpoint.left;
    ctrl.sp_right = g_setpoint.right;

    const char* modeStr = "manual";
    switch (g_mode) {
        case RobotMode::AUTONOMOUS_NAV: modeStr = "nav2"; break;
        default: break;
    }

    uint8_t payload[Cfg::MAX_FRAME_PAYLOAD];
    const size_t n = g_sensors.buildPayload(payload, sizeof(payload), modeStr, ctrl);
    if (n > 0) {
        g_link.sendTelemetry(payload, n);
    }
}

static void runStatusLed() {
    static uint32_t last = 0;
    const uint32_t now = millis();
    if (g_link.lastRxMs() == 0) {
        digitalWrite(Pins::STATUS_LED, HIGH);
        return;
    }
    const uint32_t period = g_watchdogTriggered ? 100 : 500;
    if (now - last >= period) {
        last = now;
        digitalWrite(Pins::STATUS_LED, !digitalRead(Pins::STATUS_LED));
    }
}

// ==============================================================
// Arduino
// ==============================================================

void ControlTaskCode(void * pvParameters);
void TelemetryTaskCode(void * pvParameters);

void setup() {
    pinMode(Pins::STATUS_LED, OUTPUT);
    digitalWrite(Pins::STATUS_LED, HIGH);

    //Serial.begin(Cfg::SERIAL_BAUD);  // TODO: debug temporal, quitar — Serial2 es para la Jetson

    g_link.begin();
    g_motors.begin();
    g_sensors.begin();

    g_link.onMotorCmd   (&onMotorCmd,     nullptr);
    g_link.onBrake      (&onBrake,        nullptr);
    g_link.onHeartbeat  (&onHeartbeat,    nullptr);
    g_link.onPidParam   (&onPidParam,     nullptr);
    g_link.onModeCmd    (&onModeCmd,      nullptr);
    g_link.onWheelVelCmd(&onWheelVelCmd,  nullptr);

    g_link.sendHello();

    xTaskCreatePinnedToCore(
      ControlTaskCode,
      "Control",
      10000,
      NULL,
      2,          /* Priority 2: above Arduino loop task (1) */
      &ControlTaskHandle,
      0);

    xTaskCreatePinnedToCore(
      TelemetryTaskCode,
      "Telemetry",
      10000,
      NULL,
      2,          /* Priority 2: above Arduino loop task (1) on Core 1 */
      &TelemetryTaskHandle,
      1);
}

void loop() {
    
}

//Run control
void ControlTaskCode( void * pvParameters ){
  const TickType_t periodo      = pdMS_TO_TICKS(20); // Control se ejecuta cada 20ms
  TickType_t       lastWakeTime = xTaskGetTickCount();
  for(;;){
    if (g_mode == RobotMode::AUTONOMOUS_NAV) {
        runVelocityControl(0.02f);
    }
    vTaskDelayUntil(&lastWakeTime, periodo);
  }
}

//Run Telemetry
void TelemetryTaskCode( void * pvParameters ){
  for(;;){
    g_link.tick();
    runWatchdog();
    runTelemetry();
    runStatusLed();
    delay(0);
  }
}
