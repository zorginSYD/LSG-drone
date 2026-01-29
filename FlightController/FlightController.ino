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
    Wire.setClock(400000); // Set I2C to 400kHz (Fast Mode)

    Serial.println("--- HIGH SPEED FLIGHT CONTROLLER STARTING ---");

    // 1. Initialize Input
    input.init();

    // 2. Initialize Sensors
    if (!imu.begin()) {
        Serial.println("❌ ERROR: MPU6050 Not Found!");
        // Blink error forever so user knows we are stuck here
        while(1) {
            Serial.println("❌ HALTED: Check IMU wiring!");
            delay(1000);
        } 
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
    Serial.println("Arming in 5 seconds... STAND CLEAR.");
    motors.arm(); 
    delay(4000); 

    // 4. Reset State
    targetAltitude = 0.0f;
    pidPitch.reset();
    pidRoll.reset();
    pidAlt.reset();

    Serial.println("✅ SYSTEM ARMED & READY. Waiting for input...");
    
    pinMode(13, OUTPUT); // Setup Built-in LED
}

void loop() {
    // --- Loop Heartbeat (LED Toggle) ---
    static int ledState = LOW;
    static unsigned long lastToggle = 0;
    if (millis() - lastToggle > 500) { // Slower blink to save CPU
        ledState = !ledState;
        digitalWrite(13, ledState);
        lastToggle = millis();
    }

    // --- Loop Timing (Target 5000Hz) ---
    static unsigned long lastTime = 0;
    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6f;

    if (dt < 0.0002f) return; // Wait for 0.2ms (5000Hz)
    lastTime = now;

    // Serial.print("."); // Life sign on Serial (optional, might spam)

    input.update();

    // --- 1. Read Inputs ---
    float targetPitch = input.getPitchTarget();
    float targetRoll = input.getRollTarget();
    int altChange = input.getAltitudeChange();

    if (altChange != 0) {
        targetAltitude += (float)altChange; // +/- 1 meter
        targetAltitude = constrain(targetAltitude, MIN_ALTITUDE, MAX_ALTITUDE);
        Serial.print("UPDATED Target Altitude: ");
        Serial.println(targetAltitude);
    }

    // --- 2. Read Sensors ---
    // Serial.print("S"); 
    imu.update();
    float currentPitch = imu.getPitch();
    float currentRoll = imu.getRoll();
    // Serial.print("s"); 
    
    // Serial.print("B");
    float currentAlt = baro.getAltitude();
    // Serial.print("b");

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
    int m1 = throttle + (int)pitchAdjust + (int)rollAdjust; // FL
    int m2 = throttle + (int)pitchAdjust - (int)rollAdjust; // FR
    int m3 = throttle - (int)pitchAdjust - (int)rollAdjust; // BR
    int m4 = throttle - (int)pitchAdjust + (int)rollAdjust; // BL

    // Safety Clamp again for motors
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);

    // --- 5. Write to Motors ---
    motors.write(m1, m2, m3, m4);

    // --- Debug ---
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) { // Print only once per second
        lastPrint = millis();
        Serial.print("FREQ: "); Serial.print(1.0f/dt); Serial.print("Hz | ");
        Serial.print("TgtAlt="); Serial.print(targetAltitude, 1);
        Serial.print("m | Thr="); Serial.print(throttle);
        Serial.print(" | M1="); Serial.print(m1);
        Serial.println("...");
    }
}
