#include "Input.h"

const float Input::STEP_SIZE = 5.0f;

Input::Input() 
    : _last_up_state(HIGH), _last_down_state(HIGH), 
      _pitchTarget(0.0f), _rollTarget(0.0f), _altChange(0), _failsafe(false) 
{}

void Input::init() {
    // Pins are no longer primary input, but we keep them initialized just in case
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_FAIL, INPUT_PULLUP);
}

void Input::update() {
    if (Serial.available() > 0) {
        char raw = Serial.read();
        char c = toupper(raw);
        
        Serial.print("DEBUG INPUT: Received '");
        Serial.print(raw);
        Serial.println("'");

        switch (c) {
            case 'W': // Pitch Forward (Nose Down) -> Decrease Pitch Target
                _pitchTarget -= STEP_SIZE;
                break;
            case 'S': // Pitch Back (Nose Up) -> Increase Pitch Target
                _pitchTarget += STEP_SIZE;
                break;
            case 'A': // Roll Left -> Decrease Roll Target
                _rollTarget -= STEP_SIZE;
                break;
            case 'D': // Roll Right -> Increase Roll Target
                _rollTarget += STEP_SIZE;
                break;
            case 'I': // Increase Altitude/Thrust
                _altChange = 1;
                break;
            case 'K': // Decrease Altitude/Thrust
                _altChange = -1;
                break;
            case ' ': // Spacebar -> Level Out
            case 'X':
                _pitchTarget = 0.0f;
                _rollTarget = 0.0f;
                break;
        }

        // Clamp targets
        if (_pitchTarget > MAX_TILT) _pitchTarget = MAX_TILT;
        if (_pitchTarget < -MAX_TILT) _pitchTarget = -MAX_TILT;
        if (_rollTarget > MAX_TILT) _rollTarget = MAX_TILT;
        if (_rollTarget < -MAX_TILT) _rollTarget = -MAX_TILT;

        // Debug feedback
        Serial.print("CMD: "); Serial.print(c);
        Serial.print(" | P: "); Serial.print(_pitchTarget);
        Serial.print(" R: "); Serial.println(_rollTarget);
    }
}

float Input::getPitchTarget() {
    return _pitchTarget;
}

float Input::getRollTarget() {
    return _rollTarget;
}

int Input::getAltitudeChange() {
    int change = _altChange;
    _altChange = 0; // Reset after reading (pulse)
    return change;
}

bool Input::isFailsafePressed() {
    return digitalRead(PIN_BTN_FAIL) == LOW;
}
