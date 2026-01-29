#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>

class Input {
public:
    Input();
    void init();
    void update(); // Process Serial input

    float getPitchTarget();
    float getRollTarget();
    int getAltitudeChange(); // +1 for up, -1 for down, 0 for none
    bool isFailsafePressed();

private:
    float _pitchTarget;
    float _rollTarget;
    int _altChange;
    bool _failsafe;

    static const int MAX_TILT = 30;
    static const float STEP_SIZE;

    // Pin Definitions
    static const int PIN_JOY_X = A0;
    static const int PIN_JOY_Y = A1;
    static const int PIN_BTN_UP = 2;
    static const int PIN_BTN_DOWN = 3;
    static const int PIN_BTN_FAIL = 4;

    // State Variables
    int _last_up_state;
    int _last_down_state;
};

#endif
