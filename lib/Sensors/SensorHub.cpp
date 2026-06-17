#include "SensorHub.h"
#include "Odometry.h"
#include <string.h>

SensorHub::SensorHub(Capbot::encoderPins encPins, pcnt_unit_t encUnitLeft,
    pcnt_unit_t encUnitRight, uint8_t tofXshut1, uint8_t tofXshut2,
    uint16_t filter) :
    encL_(encUnitLeft, encPins.leftA, encPins.leftB, filter),
    encR_(encUnitRight, encPins.rightA, encPins.rightB, filter),
    tof_(tofXshut1, tofXshut2) {
}

void SensorHub::begin() {
    encL_.begin();
    encR_.begin();
    imu_.begin();
    tof_.begin();
    Serial.println("SensorHub initialized");
}

void SensorHub::sample() {
    last_.enc_left         = encL_.read();
    last_.enc_right        = encR_.read();
    last_.vel_left_cps     = encL_.computeCountsPerSec();
    last_.vel_right_cps    = encR_.computeCountsPerSec();
    last_.imu_accel        = imu_.readAccel();
    last_.imu_gyro         = imu_.readGyro();
    last_.imu_mag          = imu_.readMag();
    imu_.readEuler(last_.imu_euler);
    last_.tof_sensor1_mm   = tof_.readSensor1();
    last_.tof_sensor2_mm   = tof_.readSensor2();
    last_.uptime_ms        = millis();
}

void SensorHub::feedMotorStatus(int16_t leftPwm, int16_t rightPwm, bool braking) {
    last_.motor_pwm_left  = leftPwm;
    last_.motor_pwm_right = rightPwm;
    last_.braking         = braking;
}

size_t SensorHub::buildPayload(uint8_t* out, size_t out_cap, const StateEstimate& state,
                               bool autonomousMode, const ControlTelemetry& ctrl) {
    // Telemetría binaria, little-endian, TAMAÑO FIJO.
    // El layout DEBE mantenerse en sync con el decoder de la Jetson (_TELEM_FMT).
    //   u8  schema(=1)
    //   u8  mode (0=manual, 1=auto)
    //   i16 pwm_left, pwm_right
    //   f32 odo:   x, y, a, v, w
    //   f32 sp:    x, y, a, v, w
    //   f32 error: pos, ang
    //   u16 tof:   t1, t2
    static constexpr size_t PACKET_SIZE = 58;
    static_assert(PACKET_SIZE <= Cfg::MAX_FRAME_PAYLOAD,
                  "telemetry packet exceeds frame payload");
    if (out_cap < PACKET_SIZE) return 0;

    const float dx = ctrl.sp_x - state.x;
    const float dy = ctrl.sp_y - state.y;
    const float err_pos = sqrtf(dx * dx + dy * dy);
    const float err_ang = atan2f(dy, dx) * RAD_TO_DEG - state.theta;

    size_t o = 0;
    auto put_u8  = [&](uint8_t  v){ out[o] = v; o += 1; };
    auto put_i16 = [&](int16_t  v){ memcpy(out + o, &v, 2); o += 2; };
    auto put_u16 = [&](uint16_t v){ memcpy(out + o, &v, 2); o += 2; };
    auto put_f32 = [&](float    v){ memcpy(out + o, &v, 4); o += 4; };

    put_u8(1);                          // schema
    put_u8(autonomousMode ? 1 : 0);     // mode
    put_i16(last_.motor_pwm_left);
    put_i16(last_.motor_pwm_right);
    put_f32(state.x); put_f32(state.y); put_f32(state.theta); put_f32(state.v); put_f32(state.omega);
    put_f32(ctrl.sp_x); put_f32(ctrl.sp_y); put_f32(ctrl.sp_ang); put_f32(ctrl.sp_v); put_f32(ctrl.sp_w);
    put_f32(err_pos); put_f32(err_ang);
    put_u16(last_.tof_sensor1_mm); put_u16(last_.tof_sensor2_mm);

    return o;  // siempre 58
}