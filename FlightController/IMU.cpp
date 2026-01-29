#include "IMU.h"
#include <Arduino.h>

IMU::IMU() : _pitch_deg(0.0f), _roll_deg(0.0f), _last_us(0) {}

bool IMU::begin() {
    if (!mpu.begin()) {
        return false;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    _last_us = micros();
    return true;
}

void IMU::update() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now_us = micros();
    float dt = (now_us - _last_us) * 1e-6f;
    _last_us = now_us;
    // clamp weird dt spikes
    if (dt <= 0.0f || dt > 0.1f) dt = 0.004f;

    // Accel angles
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float accel_pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
    float accel_roll  = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

    // Gyro rates
    float gyro_pitch_rate = g.gyro.x * 180.0f / PI;
    float gyro_roll_rate  = g.gyro.y * 180.0f / PI;

    // Complementary filter
    _pitch_deg = CF_ALPHA * (_pitch_deg + gyro_pitch_rate * dt) + (1.0f - CF_ALPHA) * accel_pitch;
    _roll_deg  = CF_ALPHA * (_roll_deg  + gyro_roll_rate  * dt) + (1.0f - CF_ALPHA) * accel_roll;
}

float IMU::getPitch() { return _pitch_deg; }
float IMU::getRoll() { return _roll_deg; }
