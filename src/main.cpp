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
//   6. Crea 3 tareas FreeRTOS: Control (Core 0, 20ms), Telemetry (Core 1,
//      link con la Jetson) e ImuSample (Core 1, prioridad más baja — ver
//      ImuTaskCode). El gyro vive en su propia tarea a propósito: su I2C es
//      bloqueante, y si compartiera tarea con el link a la Jetson un bus
//      lento podría demorar/perder frames de telemetría o comandos entrantes.
//
// TelemetryTaskCode (loop):
//   1. JetsonLink.tick()  -> procesa RX y dispara callbacks
//   2. Watchdog: si !hb y lastRx > JETSON_WATCHDOG_MS -> brake() y sale a manual
//   3. Cada TELEMETRY_PERIOD_MS: sample (encoders) + odometría + sendTelemetry
//   4. Pequeño yield para no hambrear tareas RTOS de fondo
//
// ImuTaskCode: muestrea el gyro cada IMU_SAMPLE_PERIOD_MS (ver Config.h),
// desacoplado del loop de arriba.

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
//   leftWheelPid / rightWheelPid : rad/s -> esfuerzo motor [-32767, 32767]
// PID posicional {kp, ki, kd, outMin, outMax, maxIntegral}.
// PLACEHOLDER: puntos de partida a escala física (error en rad/s, salida en
// counts PWM). El feedforward carga con el grueso del esfuerzo; el PID sólo
// corrige el residuo. Retunear en hardware (Fase 3) vía PID_PARAM.
//   kp = 1500 : 1 rad/s de error -> ~4.6% del duty máximo
//   ki = 500  : corrige error estacionario sin dominar la respuesta
//   maxIntegral = 65 : ki*maxIntegral ~ outMax (anti-windup por clamping)
//
// kStatic: esfuerzo de fricción (offset). Punto de partida = breakaway
// medido en banco (12812) menos margen (fricción cinética < estática).
// kV: pendiente PWM/(rad/s) en régimen. PLACEHOLDER hasta la rampa de
// caracterización (Fase 2). Ambos tunables en caliente vía PID_PARAM
// (param_id 3 y 4).
static const Controlador::Config DEFAULT_CTRL_CFG = {
    { 1500.0f, 500.0f, 0.0f, -32767.0f*0.7 , 32767.0f*0.7 , 65.0f },  // leftWheelPid
    { 1500.0f, 500.0f, 0.0f, -32767.0f*0.7 , 32767.0f*0.7 , 65.0f },  // rightWheelPid
    32767.0f,   // maxMotorOutput
    11500.0f,   // leftKStatic
    11500.0f,   // rightKStatic
    1500.0f,    // leftKV
    1500.0f     // rightKV
};

// ---- Instancias globales ----
static JetsonLink   g_link;
static MotorDriver  g_motors(leftMotorPins, rightMotorPins, Pins::LEDC_CH_LEFT, Pins::LEDC_CH_RIGHT);
static SensorHub    g_sensors(encPins, PCNT_UNIT_0, PCNT_UNIT_1, 100);
static Controlador  g_controller(DEFAULT_CTRL_CFG);
// Odometría on-board (encoders + IMU): sólo estimación de estado para
// telemetría, no interviene en el control por rueda.
static Odometry     g_odometry;

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
TaskHandle_t ImuTaskHandle;
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
// param_id 0-2: kp/ki/kd. 3-4: kStatic/kV del feedforward de fricción de esa
// rueda (no son del PID en sí, pero reusan el canal para tunear en caliente).
static void onPidParam(uint8_t ctrl_id, uint8_t param_id, float value, void* /*ctx*/) {
    if (ctrl_id > 1) return;
    const bool left = (ctrl_id == 0);

    Controlador::Config cfg = g_controller.config();
    PidController::Config* pid = left ? &cfg.leftWheelPid : &cfg.rightWheelPid;
    switch (param_id) {
        case 0: pid->kp = value; break;
        case 1: pid->ki = value; break;
        case 2: pid->kd = value; break;
        case 3: (left ? cfg.leftKStatic : cfg.rightKStatic) = value; break;
        case 4: (left ? cfg.leftKV      : cfg.rightKV)      = value; break;
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
    // g_wheelSp actúa como buffer: el PID sigue aplicando el ÚLTIMO setpoint
    // recibido mientras la Jetson calcula el siguiente (nav2 publica a ~5 Hz,
    // este lazo corre a 50 Hz), así el control es continuo y sin tirones.
    if (g_wheelSp.lastMs == 0) {
        // Recién entrados a NAV: sin setpoint todavía, quieto.
        if (!g_motors.isBraking()) g_motors.brake();
        return;
    }
    if (millis() - g_wheelSp.lastMs > Cfg::NAV_VEL_TIMEOUT_MS) {
        // Fail-safe: la Jetson dejó de mandar VEL_CMD demasiado tiempo.
        // Vaciamos el buffer y frenamos, pero nos quedamos en AUTONOMOUS_NAV
        // a la espera de un setpoint fresco.
        g_wheelSp.left  = 0.0f;
        g_wheelSp.right = 0.0f;
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

    // feedMotorStatus() antes de sample(): ImuTaskCode usa el PWM ya cargado
    // para decidir si el vehículo está quieto (descarta el gyro ese ciclo).
    g_sensors.feedMotorStatus(g_motors.leftPwm(), g_motors.rightPwm(), g_motors.isBraking());
    g_sensors.sample();

    // Odometría a partir de la muestra recién tomada (encoders + IMU).
    const StateEstimate odo = g_odometry.update(g_sensors.last());

    SensorHub::ControlTelemetry ctrl;
    ctrl.sp_left  = g_setpoint.left;
    ctrl.sp_right = g_setpoint.right;

    const char* modeStr = "manual";
    switch (g_mode) {
        case RobotMode::AUTONOMOUS_NAV: modeStr = "nav2"; break;
        default: break;
    }

    uint8_t payload[Cfg::MAX_FRAME_PAYLOAD];
    const size_t n = g_sensors.buildPayload(payload, sizeof(payload), odo, modeStr, ctrl);
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
void ImuTaskCode(void * pvParameters);

void setup() {
    pinMode(Pins::STATUS_LED, OUTPUT);
    digitalWrite(Pins::STATUS_LED, HIGH);

    //Serial.begin(Cfg::SERIAL_BAUD);  // TODO: debug temporal, quitar — Serial2 es para la Jetson

    g_link.begin();
    g_motors.begin();
    g_sensors.begin();
    g_odometry.begin();

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

    xTaskCreatePinnedToCore(
      ImuTaskCode,
      "ImuSample",
      10000,      /* Alineado con las otras dos tareas: margen de sobra para
                     medianFilter() (array local de IMU_MEDIAN_WINDOW floats)
                     y el driver I2C, corriendo sostenido a alta cadencia. */
      NULL,
      1,          /* Priority 1: por debajo de Control/Telemetry (2). El I2C
                     del gyro es una llamada bloqueante; esta prioridad más
                     baja garantiza que nunca le robe CPU al link con la
                     Jetson ni al PID si llegara a demorarse. */
      &ImuTaskHandle,
      1);         /* Core 1, junto a Telemetry: Core 0 se deja libre para el
                     PID de 20ms (ControlTaskCode), más sensible a timing. */
}

void loop() {
vTaskDelete(NULL);
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
    delay(1);
  }
}

// Muestreo del gyro en su propia tarea, desacoplado del link con la Jetson.
// Cadencia fija (Cfg::IMU_SAMPLE_PERIOD_MS) vía vTaskDelayUntil, igual que
// ControlTaskCode: así IMU_MEDIAN_WINDOW/ROLLING_WINDOW representan una
// ventana de tiempo razonable (ver Config.h) sin arriesgar bloquear
// TelemetryTaskCode si el I2C del gyro se demora.
void ImuTaskCode( void * pvParameters ){
  const TickType_t periodo      = pdMS_TO_TICKS(Cfg::IMU_SAMPLE_PERIOD_MS);
  TickType_t       lastWakeTime = xTaskGetTickCount();
  for(;;){
    g_sensors.sampleImu();
    vTaskDelayUntil(&lastWakeTime, periodo);
  }
}
