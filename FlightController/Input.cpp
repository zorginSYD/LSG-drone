#include "Input.h"

Input::Input() : _last_up_state(HIGH), _last_down_state(HIGH) {}

void Input::init() {
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_FAIL, INPUT_PULLUP);
}

float Input::getPitchTarget() {
    int val = analogRead(PIN_JOY_Y);
    // Deadzone around center (512)
    if (abs(val - 512) < 20) val = 512;

    // Map 0..1023 to -30..30 degrees
    return (val - 512) / 512.0f * (float)MAX_TILT;
}

float Input::getRollTarget() {
    int val = analogRead(PIN_JOY_X);
    // Deadzone around center (512)
    if (abs(val - 512) < 20) val = 512;

    // Map 0..1023 to -30..30 degrees
    return (val - 512) / 512.0f * (float)MAX_TILT;
}

int Input::getAltitudeChange() {
    int change = 0;

    int upState = digitalRead(PIN_BTN_UP);
    int downState = digitalRead(PIN_BTN_DOWN);

    // Simple edge detection (falling edge = press)
    // We assume this function is called often enough (e.g. 50Hz+)
    static unsigned long lastAction = 0;
    unsigned long now = millis();

    // rudimentary debounce by time gating actions
    if (now - lastAction > 200) {
        if (upState == LOW && _last_up_state == HIGH) {
            change = 1;
            lastAction = now;
        }
        else if (downState == LOW && _last_down_state == HIGH) {
            change = -1;
            lastAction = now;
        }
    }

    _last_up_state = upState;
    _last_down_state = downState;

    return change;
}

bool Input::isFailsafePressed() {
    return digitalRead(PIN_BTN_FAIL) == LOW;
}
