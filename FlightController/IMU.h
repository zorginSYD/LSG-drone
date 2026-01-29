#ifndef IMU_H
#define IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class IMU {
public:
    IMU();
    bool begin();
    void update();
    float getPitch();
    float getRoll();

private:
    Adafruit_MPU6050 mpu;
    float _pitch_deg;
    float _roll_deg;
    unsigned long _last_us;

    // Config
    const float CF_ALPHA = 0.98f;
};

#endif
