#include <Wire.h>
#include "Motors.h"
#include "IMU.h"
#include "Barometer.h"
#include "Input.h"
#include "PID.h"

// --- Global Objects ---
Motors motors;
IMU imu;
Barometer baro;
Input input;

// --- PID Controllers ---
// Pitch/Roll: Tunings derived from original code (P=2.0)
// Added some I and D placeholders, but kept low/zero for safety based on legacy behavior.
PID pidPitch(2.0, 0.0, 0.0, 400.0); // Limit adjustment to +/- 400us
PID pidRoll(2.0, 0.0, 0.0, 400.0);

// Altitude: Needs tuning.
// P=100: 1m error => 100us adjustment.
// I=50:  Accumulate error to reach hover throttle.
// D=20:  Dampen vertical oscillation.
PID pidAlt(100.0, 50.0, 20.0, 500.0); // Limit adjustment to +/- 500us

// --- State Variables ---
float targetAltitude = 0.0f;
const float MIN_ALTITUDE = 0.0f;
const float MAX_ALTITUDE = 50.0f; // Safety ceiling in meters

// Estimate hover throttle (will be corrected by PID I-term)
int baseThrottle = 1300;

// --- Failsafe Function ---
// Requested to be implemented but not active in loop
void land() {
    // Simple strategy: Set target to ground (0)
    // The altitude PID will handle the descent.
    targetAltitude = 0.0f;
    Serial.println("FAILSAFE: Landing initiated.");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("--- FLIGHT CONTROLLER STARTING ---");

    // 1. Initialize Input
    input.init();

    // 2. Initialize Sensors
    if (!imu.begin()) {
        Serial.println("❌ ERROR: MPU6050 Not Found!");
        while(1); // Halt
    }
    Serial.println("✅ MPU6050 Ready.");

    if (!baro.begin()) {
        Serial.println("⚠️ WARNING: BME280 Not Found! Altitude Hold disabled.");
        // We could halt, but for robustness let's continue.
        // However, altitude control will be garbage without it.
    } else {
        Serial.println("✅ BME280 Ready.");
    }

    // 3. Initialize Motors
    Serial.println("Initializing Motors...");
    motors.init();
    Serial.println("Arming in 5 seconds...");
    motors.arm(); // Note: This has a small delay in class, we might want longer here
    delay(4000);  // Add 4s to the 1s internal delay = 5s total

    // 4. Reset State
    targetAltitude = 0.0f;
    pidPitch.reset();
    pidRoll.reset();
    pidAlt.reset();

    Serial.println("✅ SYSTEM ARMED & READY");
}

void loop() {
    // --- Loop Timing (250Hz) ---
    static unsigned long lastTime = 0;
    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6f;

    if (dt < 0.004f) return; // Wait for 4ms
    lastTime = now;

    // --- 1. Read Inputs ---
    float targetPitch = input.getPitchTarget();
    float targetRoll = input.getRollTarget();
    int altChange = input.getAltitudeChange();

    if (altChange != 0) {
        targetAltitude += (float)altChange; // +/- 1 meter
        targetAltitude = constrain(targetAltitude, MIN_ALTITUDE, MAX_ALTITUDE);
        Serial.print("New Target Altitude: ");
        Serial.print(targetAltitude);
        Serial.println(" m");
    }

    // --- 2. Read Sensors ---
    imu.update();
    float currentPitch = imu.getPitch();
    float currentRoll = imu.getRoll();
    float currentAlt = baro.getAltitude();

    // --- 3. Run Control Algorithms ---

    // Altitude PID
    // Calculate throttle adjustment based on altitude error
    // If current < target, error is positive -> throttle increases
    float altOutput = pidAlt.compute(targetAltitude, currentAlt, dt);

    // Base throttle + PID output
    int throttle = baseThrottle + (int)altOutput;

    // Clamp total throttle for safety
    throttle = constrain(throttle, 1100, 1900);

    // Attitude PIDs
    float pitchAdjust = pidPitch.compute(targetPitch, currentPitch, dt);
    float rollAdjust = pidRoll.compute(targetRoll, currentRoll, dt);

    // --- 4. Motor Mixing ---
    // Quad X configuration (Standard)
    // FL (CW)  FR (CCW)
    // BL (CCW) BR (CW)
    // Note: Direction depends on prop/motor install, but mixing logic assumes:
    // Pitch forward -> Back motors speed up, Front slow down
    // Roll right -> Left motors speed up, Right slow down

    // From legacy code:
    // FL = Base + Pitch + Roll
    // FR = Base + Pitch - Roll
    // BR = Base - Pitch - Roll
    // BL = Base - Pitch + Roll
    // Let's verify signs:
    // If Pitch (Forward) is positive input?
    // Usually Pitch Back is positive.
    // Legacy: `pitch_adjust = pitch_deg * P_GAIN`
    // If `pitch_deg` is positive (nose up), `pitch_adjust` is pos.
    // FL = Base + pos = Faster.
    // FR = Base + pos = Faster.
    // BR/BL = Base - pos = Slower.
    // Result: Front motors faster -> Nose goes UP.
    // So positive Pitch = Nose Up.

    int m1 = throttle + (int)pitchAdjust + (int)rollAdjust; // FL
    int m2 = throttle + (int)pitchAdjust - (int)rollAdjust; // FR
    int m3 = throttle - (int)pitchAdjust - (int)rollAdjust; // BR
    int m4 = throttle - (int)pitchAdjust + (int)rollAdjust; // BL

    // --- 5. Write to Motors ---
    motors.write(m1, m2, m3, m4);

    // --- Debug ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        lastPrint = millis();
        // Serial.print("Alt: "); Serial.print(currentAlt);
        // Serial.print(" Tgt: "); Serial.print(targetAltitude);
        // Serial.print(" Thr: "); Serial.println(throttle);
    }
}
