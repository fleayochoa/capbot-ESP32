#include "IMUSensor.h"

#include <math.h>

// Factor de conversión rad/s -> deg/s (Adafruit expone gyro en rps).
static constexpr float RPS_TO_DPS = 57.29577951308232f;   // 180/π

IMUSensor::IMUSensor(uint8_t addr, TwoWire& wire)
    : addr_(addr),
      wire_(&wire),
      // sensorID=55 es solo un identificador para Adafruit_Sensor; no afecta I2C.
      bno_(55, addr, &wire) {}

bool IMUSensor::begin(OpMode mode, bool useExternalCrystal) {
    // Adafruit_BNO055::begin():
    //   - Hace Wire.begin() si no se hizo antes
    //   - Espera ~650 ms al boot del chip y verifica CHIP_ID
    //   - Setea UNIT_SEL (gyro en rps!, accel m/s², euler deg)
    //   - Aplica el modo pedido
    // Internamente bloquea ~850 ms en frío. No es async.
    if (!bno_.begin(static_cast<adafruit_bno055_opmode_t>(mode))) {
        return false;
    }

    mode_ = mode;

    // I2C a 400 kHz (Adafruit deja el default de Wire, típicamente 100 kHz).
    // Lo subimos después del begin() para no interferir con su sondeo inicial.
    wire_->setClock(400000);

    // Cristal externo del módulo GY-BNO055.
    // Nota: setExtCrystalUse necesita estar en CONFIG mode internamente; la lib
    // ya lo maneja (hace switch a CONFIG, escribe el bit, vuelve al modo previo).
    offsets.accel_offset_x = -3;
    offsets.accel_offset_y = 43;
    offsets.accel_offset_z = -26;
    offsets.mag_offset_x = -572;
    offsets.mag_offset_y = 261;
    offsets.mag_offset_z = 106;
    offsets.gyro_offset_x = -2;
    offsets.gyro_offset_y = 0;
    offsets.gyro_offset_z = -1;
    offsets.accel_radius = 1000;
    offsets.mag_radius = 815;
    bno_.setSensorOffsets(offsets);
    bno_.setExtCrystalUse(useExternalCrystal);

    return true;
}

bool IMUSensor::isConnected() {
    // Ping I2C crudo (la lib de Adafruit no expone esto). Si el chip no
    // contesta el ACK a la dirección, endTransmission() != 0.
    wire_->beginTransmission(addr_);
    return wire_->endTransmission() == 0;
}

bool IMUSensor::setMode(OpMode m) {
    // Adafruit::setMode es void; no podemos validar. Marcamos el estado local
    // y asumimos éxito. El delay de transición (25 ms) lo hace la lib.
    bno_.setMode(static_cast<adafruit_bno055_opmode_t>(m));
    mode_ = m;
    return true;
}

// ================= Lecturas =================

void IMUSensor::copyVec(const imu::Vector<3>& src, Vec3& dst, float scale) {
    dst.x = static_cast<float>(src.x()) * scale;
    dst.y = static_cast<float>(src.y()) * scale;
    dst.z = static_cast<float>(src.z()) * scale;
}

IMUSensor::Vec3 IMUSensor::readAccel() {
    Vec3 v;
    copyVec(bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER), v);
    return v;
}

IMUSensor::Vec3 IMUSensor::readGyro() {
    Vec3 v;
    // Adafruit configura UNIT_SEL con gyro en RPS. Convertimos a DPS para
    // mantener la misma semántica del driver anterior.
    copyVec(bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE), v, RPS_TO_DPS);
    return v;
}

IMUSensor::Vec3 IMUSensor::readMag() {
    Vec3 v;
    copyVec(bno_.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER), v);
    return v;
}

IMUSensor::Vec3 IMUSensor::readLinearAccel() {
    Vec3 v;
    copyVec(bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL), v);
    return v;
}

bool IMUSensor::readGravity(Vec3& v) {
    copyVec(bno_.getVector(Adafruit_BNO055::VECTOR_GRAVITY), v);
    return true;
}

bool IMUSensor::readEuler(Euler& e) {
    // En Adafruit VECTOR_EULER devuelve (x=heading, y=roll, z=pitch) en deg.
    const imu::Vector<3> ev = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
    e.heading = static_cast<float>(ev.x());
    e.roll    = static_cast<float>(ev.y());
    e.pitch   = static_cast<float>(ev.z());
    return true;
}

float IMUSensor::readOrientation() {
    // Devuelve la orientación del robot en el plano horizontal (heading/yaw).
    sensors_event_t orientationData, gyroData;
    bno_.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    return orientationData.orientation.x;
}

bool IMUSensor::readQuaternion(Quat& q) {
    const imu::Quaternion qv = bno_.getQuat();
    q.w = static_cast<float>(qv.w());
    q.x = static_cast<float>(qv.x());
    q.y = static_cast<float>(qv.y());
    q.z = static_cast<float>(qv.z());
    return true;
}

bool IMUSensor::readCalibStatus(CalibStatus& c) {
    uint8_t sys = 0, gyr = 0, acc = 0, mag = 0;
    bno_.getCalibration(&sys, &gyr, &acc, &mag);
    c.sys = sys; c.gyr = gyr; c.acc = acc; c.mag = mag;
    return true;
}

bool IMUSensor::readTemperature(int8_t& t_c) {
    t_c = bno_.getTemp();
    return true;
}

bool IMUSensor::isFullyCalibrated() {
    return bno_.isFullyCalibrated();
}