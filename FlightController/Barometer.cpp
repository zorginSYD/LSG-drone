#include "Barometer.h"
#include <Arduino.h>

Barometer::Barometer() : _ground_pressure(1013.25f) {}

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
    }
    return ok;
}

void Barometer::tare() {
    _ground_pressure = bme.readPressure() / 100.0F;
}

float Barometer::getAltitude() {
    return bme.readAltitude(_ground_pressure);
}
