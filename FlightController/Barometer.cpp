#include "Barometer.h"
#include <Arduino.h>

Barometer::Barometer() : _ground_pressure(1013.25f), _ready(false) {}

bool Barometer::begin() {
    // Try both standard I2C addresses
    bool ok = bme.begin(0x76);
    if (!ok) {
        ok = bme.begin(0x77);
    }
    if (ok) {
        // Small delay to ensure sensor is ready
        delay(100);
        tare();
        _ready = true;
    } else {
        _ready = false;
    }
    return ok;
}

void Barometer::tare() {
    if (!_ready) return;
    _ground_pressure = bme.readPressure() / 100.0F;
}

float Barometer::getAltitude() {
    if (!_ready) return 0.0f;
    return bme.readAltitude(_ground_pressure);
}
