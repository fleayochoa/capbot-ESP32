// main.cpp — Entry point del firmware.
//
// setup():
//   1. Inicializa Serial con la Jetson
//   2. Inicializa motores (arrancan en freno por seguridad)
//   3. Inicializa sensores (encoders)
//   4. Registra callbacks en JetsonLink
//   5. Manda ESP_HELLO
//
// loop():
//   1. JetsonLink.tick()  -> procesa RX y dispara callbacks
//   2. Watchdog: si !hb y lastRx > JETSON_WATCHDOG_MS -> brake()
//   3. Cada TELEMETRY_PERIOD_MS: sample + sendTelemetry
//   4. Pequeño yield para no hambrear tareas RTOS de fondo
//
// Arquitectura: TODO el estado mutable vive en variables globales de este
// archivo. Los módulos (MotorDriver, SensorHub, JetsonLink) son clases con
// estado encapsulado; main.cpp es "sólo cableado".

#include <Arduino.h>

#include "Config.h"
#include "Pins.h"
#include "JetsonLink.h"
#include "MotorDriver.h"
#include "SensorHub.h"
#include "Odometry.h"
#include "CapTypes.h"
#include "Control.h"
// ---- Types ----
Capbot::encoderPins encPins = {
    static_cast<gpio_num_t>(Pins::ENC_LEFT_A), static_cast<gpio_num_t>(Pins::ENC_LEFT_B),
    static_cast<gpio_num_t>(Pins::ENC_RIGHT_A), static_cast<gpio_num_t>(Pins::ENC_RIGHT_B)
};
Capbot::motorPins leftMotorPins = {Pins::LEFT_IN1, Pins::LEFT_IN2, Pins::LEFT_ENA};
Capbot::motorPins rightMotorPins = {Pins::RIGHT_IN1, Pins::RIGHT_IN2, Pins::RIGHT_ENA};
// ---- Instancias globales ----
static const Controlador::Config DEFAULT_CTRL_CFG = {
    { 1.5f,     0.0f, 0.0f, -0.5f,    0.5f,    1.0f    },  // linearPosPid
    { 20000.0f, 0.0f, 0.0f, -32767.0f, 32767.0f, 50000.0f },  // linearVelPid
    { 2.0f,     0.0f, 0.0f, -90.0f,   90.0f,  180.0f   },  // angularPosPid
    { 300.0f,   0.0f, 0.0f, -32767.0f, 32767.0f, 50000.0f },  // angularVelPid
    0.02f, 2.0f, 32767.0f   // thetaPositionTolerance, thetaAngleTolerance, maxMotorOutput
};

static JetsonLink   g_link;
static MotorDriver  g_motors(leftMotorPins, rightMotorPins, Pins::LEDC_CH_LEFT, Pins::LEDC_CH_RIGHT);
static SensorHub    g_sensors(encPins, PCNT_UNIT_0, PCNT_UNIT_1, Pins::TOF_XSHUT1, Pins::TOF_XSHUT2, 100);
static Odometry     g_odometry;
static Controlador  g_controller(DEFAULT_CTRL_CFG);

// ---- Estado de control ----
static struct {
    float x      = 0.0f;
    float y      = 0.0f;
    float angPos = 0.0f;
} g_setpoint;

// Setpoint de velocidad proveniente de ROS2 /cmd_vel (via VEL_CMD).
static struct {
    float    v_mps    = 0.0f;   // m/s
    float    w_rads   = 0.0f;   // rad/s
    uint32_t lastMs   = 0;      // millis() del último VEL_CMD recibido
    bool     active   = false;  // true: modo velocidad activo
} g_vel;

static uint32_t g_lastTelemetryMs   = 0;
static bool     g_watchdogTriggered = false;

// ==============================================================
// Callbacks del JetsonLink
// ==============================================================
static void onMotorCmd(int16_t left, int16_t right, int16_t aux, void* /*ctx*/) {
    (void)aux;  // aux reservado para futuro (luces, brazo, etc.)
    g_vel.active = false;  // MOTOR_CMD toma prioridad sobre el control de velocidad
    g_motors.drive(left, right);
    g_watchdogTriggered = false;
}

static void onVelCmd(int16_t v_mms, int16_t w_mrad_s, void* /*ctx*/) {
    g_vel.v_mps  = v_mms   * 1e-3f;   // mm/s  → m/s
    g_vel.w_rads = w_mrad_s * 1e-3f;  // mrad/s → rad/s
    g_vel.lastMs = millis();
    g_vel.active = true;
    g_watchdogTriggered = false;
}

static void onBrake(void* /*ctx*/) {
    g_vel.active = false;
    g_motors.brake();
}

static void onHeartbeat(void* /*ctx*/) {
    // Heartbeat sólo refresca el timestamp de RX (lo hace el JetsonLink).
    // No tocamos motores: si estábamos frenados por emergencia, seguimos
    // frenados hasta que llegue un MOTOR_CMD real.
    g_watchdogTriggered = false;
}

// ==============================================================
// Tareas del loop
// ==============================================================
static void runWatchdog() {
    const uint32_t last = g_link.lastRxMs();
    // Caso especial: todavía NO recibimos nada desde boot. Seguimos en freno
    // (que es como arrancamos) hasta que la Jetson dé señales de vida.
    if (last == 0) {
        if (!g_motors.isBraking()) {
            g_motors.brake();
        }
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

static void runTelemetry() {
    const uint32_t now = millis();
    if (now - g_lastTelemetryMs < Cfg::TELEMETRY_PERIOD_MS) return;
    const float dt = (g_lastTelemetryMs > 0)
        ? (now - g_lastTelemetryMs) * 1e-3f
        : Cfg::TELEMETRY_PERIOD_MS * 1e-3f;
    g_lastTelemetryMs = now;

    g_sensors.sample();
    g_sensors.feedMotorStatus(
        g_motors.leftPwm(), g_motors.rightPwm(), g_motors.isBraking());

    const StateEstimate est = g_odometry.update(g_sensors.last(), true);

    // Control de velocidad para ROS2 /cmd_vel
    if (g_vel.active) {
        if (now - g_vel.lastMs > Cfg::VEL_CMD_TIMEOUT_MS) {
            // /cmd_vel dejó de llegar; detenemos suavemente
            g_vel.active = false;
            g_controller.reset();
            g_motors.brake();
        } else {
            Controlador::VelSetpoint sp { g_vel.v_mps, g_vel.w_rads };
            // omega del StateEstimate está en grados/s; el PID usa rad/s
            Controlador::State cs {
                0.0f, est.v,
                0.0f, est.omega * (M_PI / 180.0f)
            };
            const Controlador::MotorCommand mc = g_controller.computeVelocity(sp, cs, dt);
            if (mc.brake) {
                g_motors.brake();
            } else {
                g_motors.drive(static_cast<int16_t>(mc.left), static_cast<int16_t>(mc.right));
            }
        }
    }

    uint8_t payload[Cfg::MAX_FRAME_PAYLOAD];
    const size_t n = g_sensors.buildPayload(payload, sizeof(payload), est);
    if (n > 0) {
        g_link.sendTelemetry(payload, n);
    }
}

static void runStatusLed() {
    // LED parpadea según estado:
    //   - sin Jetson (lastRx == 0): siempre encendido
    //   - watchdog tripped: parpadeo rápido (100 ms)
    //   - operación normal: parpadeo lento (1 Hz)
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
    g_motors.begin();  // arranca en freno
    g_sensors.begin();
    g_odometry.begin();

    g_link.onMotorCmd (&onMotorCmd,  nullptr);
    g_link.onVelCmd   (&onVelCmd,    nullptr);
    g_link.onBrake    (&onBrake,     nullptr);
    g_link.onHeartbeat(&onHeartbeat, nullptr);

    // Anunciamos a la Jetson que arrancamos (y reiniciamos el contador
    // de seq del lado Jetson si lo implementa).
    g_link.sendHello();
}

void loop() {
    g_link.tick();
    runWatchdog();
    runTelemetry();
    runStatusLed();
    // Cedemos tiempo al scheduler RTOS (WiFi stack, housekeeping).
    // delay(0) equivale a yield() en ESP32.
    delay(0);
}