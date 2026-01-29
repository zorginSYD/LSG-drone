#ifndef MOTORS_H
#define MOTORS_H

#include <Servo.h>
#include <Arduino.h>

class Motors {
public:
    Motors();
    void init();
    void arm();
    void write(int fl, int fr, int br, int bl);
    void stop();

    static const int MIN_THROTTLE = 1000;
    static const int MAX_THROTTLE = 2000;

private:
    Servo m1, m2, m3, m4;

    const int PIN_FL = 7;
    const int PIN_FR = 9;
    const int PIN_BR = 6;
    const int PIN_BL = 5;
};

#endif
