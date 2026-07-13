#include "IMUSensor.h"

IMUSensor::IMUSensor(uint8_t addr, TwoWire& wire)
    : addr_(addr), wire_(&wire) {}

void IMUSensor::writeReg8(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(value);
    wire_->endTransmission();
}

uint8_t IMUSensor::readReg8(uint8_t reg) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->endTransmission(false);  // restart, sin soltar el bus
    wire_->requestFrom(addr_, static_cast<uint8_t>(1));
    return wire_->available() ? wire_->read() : 0;
}

bool IMUSensor::readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom(addr_, len) != len) return false;
    for (uint8_t i = 0; i < len; ++i) buf[i] = wire_->read();
    return true;
}

bool IMUSensor::begin() {
    // Ping simple: si no hay ACK en la dirección, no hay chip en el bus.
    wire_->beginTransmission(addr_);
    if (wire_->endTransmission() != 0) {
        ok_ = false;
        return false;
    }

    // Sólo informativo: 0x68=MPU6050, 0x70=MPU6500 (otras variantes de la
    // familia pueden diferir). No se usa para rechazar el chip.
    Serial.printf("IMU WHO_AM_I=0x%02X (0x68=MPU6050, 0x70=MPU6500)\n", readReg8(REG_WHO_AM_I));

    writeReg8(REG_PWR_MGMT_1, 0x80);  // reset del dispositivo
    delay(100);
    writeReg8(REG_PWR_MGMT_1, 0x01);  // despierta; clock = PLL con giro X como referencia
    delay(10);

    writeReg8(REG_SMPLRT_DIV, 0x00);
    // DLPF_CFG=5 -> ~10 Hz de ancho de banda: más atenuación de vibración del
    // chasis que 21 Hz (DLPF_CFG=4), a costa de algo más de latencia —
    // aceptable porque esta señal no alimenta el lazo de control por rueda.
    writeReg8(REG_CONFIG, 0x05);
    // FS_SEL=1 (bits 4:3) -> ±500 °/s: cubre giros rápidos sin saturar.
    writeReg8(REG_GYRO_CONFIG, 0x08);

    ok_ = true;
    return true;
}

bool IMUSensor::isConnected() {
    wire_->beginTransmission(addr_);
    return wire_->endTransmission() == 0;
}

bool IMUSensor::readRawGyroZ(float& gz) {
    uint8_t buf[2];
    if (!readBurst(REG_GYRO_ZOUT_H, buf, sizeof(buf))) return false;

    const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    gz = (raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    return true;
}

void IMUSensor::medianFilter(float& gz) {
    medBuf_[medIdx_] = gz;
    medIdx_ = (medIdx_ + 1) % MEDIAN_WIN;
    if (!medFull_ && medIdx_ == 0) medFull_ = true;
    const uint8_t n = medFull_ ? MEDIAN_WIN : medIdx_;

    // Copia + insertion sort: n <= MEDIAN_WIN (4), de sobra rápido para
    // correr una vez por muestra de telemetría.
    float sorted[MEDIAN_WIN];
    for (uint8_t i = 0; i < n; ++i) sorted[i] = medBuf_[i];
    for (uint8_t i = 1; i < n; ++i) {
        const float key = sorted[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = key;
    }

    // n par (incluido el caso de ventana llena, 4): promedio de los dos
    // valores centrales. n impar (mientras se llena el buffer): el del medio.
    if (n % 2 == 0) {
        gz = 0.5f * (sorted[n / 2 - 1] + sorted[n / 2]);
    } else {
        gz = sorted[n / 2];
    }
}

void IMUSensor::rollingMean(float& gz) {
    gzBuf_[rollIdx_] = gz;
    rollIdx_ = (rollIdx_ + 1) % ROLL_WIN;
    if (!rollFull_ && rollIdx_ == 0) rollFull_ = true;
    const uint8_t n = rollFull_ ? ROLL_WIN : rollIdx_;

    float sum = 0.0f;
    for (uint8_t i = 0; i < n; ++i) sum += gzBuf_[i];
    gz = sum / n;
}

bool IMUSensor::calibrate(uint16_t samples, uint16_t delayMs) {
    if (!ok_ || samples == 0) return false;

    gyroZBias_ = 0.0f;

    // double para no perder precisión al sumar miles de muestras.
    double   sum = 0;
    uint16_t okCount = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        float gz;
        if (readRawGyroZ(gz)) { sum += gz; ++okCount; }
        if (delayMs) delay(delayMs);
    }

    // Si hubo lecturas fallidas, promediar sólo sobre las válidas (dividir
    // por `samples` fijo sesgaría el bias hacia 0 cada vez que el I2C falla).
    if (okCount == 0) return false;

    gyroZBias_  = static_cast<float>(sum / okCount);
    calibrated_ = true;
    return true;
}

bool IMUSensor::read(float& gyroZ) {
    if (!ok_) {
        gyroZ = lastGyroZ_;
        lastReadOk_ = false;
        return false;
    }

    float gz;
    if (!readRawGyroZ(gz)) {
        // Falla de lectura I2C: NO empujamos nada a los buffers de
        // mediana/media móvil. Un 0 artificial contaminaría hasta
        // MEDIAN_WIN + ROLL_WIN muestras siguientes con un salto falso;
        // en cambio, mantenemos el último valor filtrado válido.
        gyroZ = lastGyroZ_;
        lastReadOk_ = false;
        return false;
    }
    lastReadOk_ = true;

    // Resta de bias de calibración (cero si no se calibró).
    gz -= gyroZBias_;

    // Montaje derecho (X adelante, Z arriba): el eje Z crudo del sensor ya
    // coincide con el marco del robot (CCW-positivo por regla de la mano
    // derecha), así que no se remapea el signo.

    // Mediana primero: rechaza picos impulsivos (spikes puntuales) que una
    // media móvil corta dejaría pasar. Media móvil después: atenúa el ruido
    // de vibración de más baja amplitud antes de que Odometry fusione esta
    // lectura con el yaw de los encoders.
    medianFilter(gz);
    rollingMean(gz);

    lastGyroZ_ = gz;
    gyroZ = gz;
    return true;
}
