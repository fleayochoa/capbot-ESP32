// main.cpp — Entry point del firmware.
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
//   2. Watchdog: si !hb y lastRx > JETSON_WATCHDOG_MS -> brake()
//   3. Cada TELEMETRY_PERIOD_MS: sample + odometría + control + sendTelemetry
//   4. Pequeño yield para no hambrear tareas RTOS de fondo

#include <Arduino.h>

#include "Config.h"
#include "Pins.h"
#include "JetsonLink.h"
#include "MotorDriver.h"
#include "SensorHub.h"
#include "Odometry.h"
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
//   linearPosPid  : metros -> setpoint m/s
//   linearVelPid  : m/s    -> esfuerzo motor [-32767, 32767]
//   angularPosPid : grados -> setpoint deg/s
//   angularVelPid : deg/s  -> esfuerzo diferencial [-32767, 32767]
static const Controlador::Config DEFAULT_CTRL_CFG = {
    { 1.5f,     0.0f, 0.0f, -0.5f,    0.5f,    1.0f    },  // linearPosPid
    { 20000.0f, 0.0f, 0.0f, -32767.0f, 32767.0f, 50000.0f },  // linearVelPid
    { 2.0f,     0.0f, 0.0f, -90.0f,   90.0f,  180.0f   },  // angularPosPid
    { 300.0f,   0.0f, 0.0f, -32767.0f, 32767.0f, 50000.0f },  // angularVelPid
    0.02f, 2.0f, 32767.0f   // thetaPositionTolerance, thetaAngleTolerance, maxMotorOutput
};

// ---- Instancias globales ----
static JetsonLink   g_link;
static MotorDriver  g_motors(leftMotorPins, rightMotorPins, Pins::LEDC_CH_LEFT, Pins::LEDC_CH_RIGHT);
static SensorHub    g_sensors(encPins, PCNT_UNIT_0, PCNT_UNIT_1, 100);
static Odometry     g_odometry;
static Controlador  g_controller(DEFAULT_CTRL_CFG);

// ---- Estado de control ----
static struct {
    float x      = 0.0f;
    float y      = 0.0f;
    float angPos = 0.0f;
} g_setpoint;

static bool     g_autonomousMode    = false;
static uint32_t g_lastTelemetryMs   = 0;
static bool     g_watchdogTriggered = false;

// ==============================================================
// Callbacks del JetsonLink
// ==============================================================
static void onMotorCmd(int16_t left, int16_t right, int16_t aux, void* /*ctx*/) {
    (void)aux;
    if (!g_autonomousMode) {
        g_motors.drive(left, right);
    }
    g_watchdogTriggered = false;
}

static void onBrake(void* /*ctx*/) {
    g_motors.brake();
}

static void onHeartbeat(void* /*ctx*/) {
    g_watchdogTriggered = false;
}

static void onPidParam(uint8_t ctrl_id, uint8_t param_id, float value, void* /*ctx*/) {
    Controlador::Config cfg = g_controller.config();
    PidController::Config* pid = nullptr;
    switch (ctrl_id) {
        case 0: pid = &cfg.linearPosPid;   break;
        case 1: pid = &cfg.linearVelPid;   break;
        case 2: pid = &cfg.angularPosPid;  break;
        case 3: pid = &cfg.angularVelPid;  break;
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

static void onSetpointComp(uint8_t comp_id, float value, void* /*ctx*/) {
    switch (comp_id) {
        case 0: g_setpoint.x      = value; break;
        case 1: g_setpoint.y      = value; break;
        case 2: g_setpoint.angPos = value; break;
        default: break;
    }
}

static void onModeCmd(uint8_t mode, void* /*ctx*/) {
    g_autonomousMode = (mode != 0);
    if (!g_autonomousMode) {
        g_controller.reset();
        g_motors.brake();
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
            g_motors.brake();
            g_watchdogTriggered = true;
        }
    }
}

static void runControl(float dt) {
    const StateEstimate& odo = g_odometry.state();

    Controlador::State st;
    st.linearPosition  = odo.x;
    st.linearVelocity  = odo.v;
    st.angularPosition = odo.theta;
    st.angularVelocity = odo.omega;

    Controlador::Setpoint sp;
    sp.linearPosition  = g_setpoint.x;
    sp.angularPosition = g_setpoint.angPos;

    const Controlador::MotorCommand cmd = g_controller.compute(sp, st, dt);
    if (cmd.brake) {
        g_motors.brake();
    } else {
        g_motors.drive(static_cast<int16_t>(cmd.left), static_cast<int16_t>(cmd.right));
    }
}

static void runTelemetry() {
    const uint32_t now = millis();
    if (now - g_lastTelemetryMs < Cfg::TELEMETRY_PERIOD_MS) return;
    const float dt = static_cast<float>(now - g_lastTelemetryMs) / 1000.0f;
    g_lastTelemetryMs = now;

    g_sensors.sample();
    g_sensors.feedMotorStatus(g_motors.leftPwm(), g_motors.rightPwm(), g_motors.isBraking());
    g_odometry.update(g_sensors.last(), false);

    if (g_autonomousMode) {
        runControl(dt);
    }

    uint8_t payload[Cfg::MAX_FRAME_PAYLOAD];
    const size_t n = g_sensors.buildPayload(payload, sizeof(payload), g_odometry.state());
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
void setup() {
    pinMode(Pins::STATUS_LED, OUTPUT);
    digitalWrite(Pins::STATUS_LED, HIGH);

    g_link.begin();
    g_motors.begin();
    g_sensors.begin();
    g_odometry.begin();

    g_link.onMotorCmd  (&onMotorCmd,     nullptr);
    g_link.onBrake     (&onBrake,        nullptr);
    g_link.onHeartbeat (&onHeartbeat,    nullptr);
    g_link.onPidParam  (&onPidParam,     nullptr);
    g_link.onSetpoint  (&onSetpointComp, nullptr);
    g_link.onModeCmd   (&onModeCmd,      nullptr);

    g_link.sendHello();
}

void loop() {
    g_link.tick();
    runWatchdog();
    runTelemetry();
    runStatusLed();
    delay(0);
}
