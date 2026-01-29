#include "Motors.h"

Motors::Motors() {}

void Motors::init() {
    m1.attach(PIN_FL, MIN_THROTTLE, MAX_THROTTLE);
    m2.attach(PIN_FR, MIN_THROTTLE, MAX_THROTTLE);
    m3.attach(PIN_BR, MIN_THROTTLE, MAX_THROTTLE);
    m4.attach(PIN_BL, MIN_THROTTLE, MAX_THROTTLE);
}

void Motors::arm() {
    // Send min throttle to arm ESCs
    write(MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE);
    // Delay is usually done in main setup to allow user to see it happening,
    // but we can put a small delay here or rely on the caller.
    // The original code had a 5s delay. We will trust the caller to manage long delays
    // if they want to print debug info, but for safety let's put a minimal delay here.
    delay(100);
}

void Motors::write(int fl, int fr, int br, int bl) {
    fl = constrain(fl, MIN_THROTTLE, MAX_THROTTLE);
    fr = constrain(fr, MIN_THROTTLE, MAX_THROTTLE);
    br = constrain(br, MIN_THROTTLE, MAX_THROTTLE);
    bl = constrain(bl, MIN_THROTTLE, MAX_THROTTLE);

    m1.writeMicroseconds(fl);
    m2.writeMicroseconds(fr);
    m3.writeMicroseconds(br);
    m4.writeMicroseconds(bl);
}

void Motors::stop() {
    write(MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE, MIN_THROTTLE);
}
