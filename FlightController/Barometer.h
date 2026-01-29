#ifndef BAROMETER_H
#define BAROMETER_H

#include <Adafruit_BME280.h>
#include <Wire.h>

class Barometer {
public:
    Barometer();
    bool begin();
    void tare();
    float getAltitude(); // Relative to start

private:
    Adafruit_BME280 bme;
    float _ground_pressure;
    bool _ready;
};

#endif
