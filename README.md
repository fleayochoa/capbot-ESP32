# ESP32 Firmware — Motor & Sensor Bridge

Firmware Arduino-sobre-PlatformIO que habla COBS+CRC16 con la Jetson Nano por
serial USB. Controla motores vía L298N y publica telemetría de sensores.

## Arquitectura

```
esp32_firmware/
├── platformio.ini
├── src/
│   └── main.cpp                  # setup() + loop(): sólo orquesta
├── include/
│   ├── Pins.h                    # Mapa de pines (editar según hardware)
│   └── Config.h                  # Constantes (baudrate, watchdog, rates)
├── test/
│   └── test_protocol/            # Tests nativos del codec
└── lib/
    ├── Protocol/                 # COBS + CRC16 (espejo de la Jetson)
    ├── Link/                     # Serial link con Jetson
    ├── Motors/                   # Driver L298N con PWM + freno activo
    └── Sensors/                  # Lectura y empaquetado de sensores
```

## Responsabilidades por capa

| Módulo        | Qué hace                                                         |
|---------------|------------------------------------------------------------------|
| `Protocol`    | Encode/decode COBS y CRC16-CCITT, framing tipo+len+payload+crc   |
| `JetsonLink`  | Lee bytes del Serial, entrega callbacks por tipo de mensaje      |
| `MotorDriver` | IN1/IN2 + ENA (PWM) por rueda, implementa freno activo L298N     |
| `SensorHub`   | Lee sensores a X Hz y arma paquete de telemetría                 |
| `main.cpp`    | Conecta callbacks, ejecuta watchdog, coordina loop               |

## Watchdog (requisito ME ms)

El firmware mantiene `lastJetsonRxMs`. En cada iteración del loop:

```cpp
if (millis() - lastJetsonRxMs > JETSON_WATCHDOG_MS) {
    motors.brake();  // freno activo L298N
}
```

La Jetson manda heartbeat cada 50 ms; el watchdog dispara a 200 ms (4 pérdidas
toleradas).

## Build / upload / monitor

```bash
pio run                      # compilar
pio run -t upload            # flashear
pio device monitor -b 921600 # ver logs
pio test -e native           # correr tests del protocolo en PC
```

## Protocolo serial (resumen)

```
STREAM = [ COBS-encoded( [type:1][len:1][payload:len][crc16:2] ) ][ 0x00 ]
```

Tipos de mensaje:

| Dirección      | Valor | Nombre       | Payload                          |
|----------------|-------|--------------|----------------------------------|
| Jetson→ESP32   | 0x10  | MOTOR_CMD    | int16 L, int16 R, int16 aux      |
| Jetson→ESP32   | 0x11  | BRAKE_ON     | (vacío)                          |
| Jetson→ESP32   | 0x12  | HEARTBEAT    | (vacío)                          |
| ESP32→Jetson   | 0x20  | TELEMETRY    | JSON UTF-8 o struct              |
| ESP32→Jetson   | 0x21  | ESP_HELLO    | (vacío, al arrancar)             |

CRC16-CCITT, poly 0x1021, init 0xFFFF. Vector de test: `CRC("123456789") == 0x29B1`.