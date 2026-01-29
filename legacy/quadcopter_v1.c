#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

// --- 1. PIN DEFINITIONS & SETTINGS ---
const int PIN_M1_FL = 7;  // Front Left
const int PIN_M2_FR = 9;  // Front Right
const int PIN_M3_BR = 6;  // Back Right
const int PIN_M4_BL = 5;  // Back Left

const int MIN_THROTTLE  = 1000;
const int MAX_THROTTLE  = 2000;
const int BASE_THROTTLE = 1300;

const float P_GAIN_PITCH = 2.0f;
const float P_GAIN_ROLL  = 2.0f;

const int MAX_TILT_ANGLE = 30;

// Complementary filter
// 0.98 = trust gyro more, 0.02 = slowly correct with accel
const float CF_ALPHA = 0.98f;

// --- 2. CREATE OBJECTS ---
Servo m1, m2, m3, m4;
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

bool imu_ok = false;
bool bme_ok = false;

static float pitch_deg = 0.0f;
static float roll_deg  = 0.0f;

static unsigned long last_us = 0;

// --- 3. HELPER FUNCTION ---
void writeAllMotors(int val) {
  m1.writeMicroseconds(val);
  m2.writeMicroseconds(val);
  m3.writeMicroseconds(val);
  m4.writeMicroseconds(val);
}

static inline bool isBad(float x) {
  return isnan(x) || isinf(x);
}

// --- 4. SETUP ---
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("--- SYSTEM STARTING ---");

  // A. Initialize MPU6050
  imu_ok = mpu.begin();
  if (!imu_ok) {
    Serial.println("❌ ERROR: MPU6050 Not Found! Check wiring.");
  } else {
    Serial.println("✅ MPU6050 Connected.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // B. Initialize BME280 (optional)
  if (bme.begin(0x76)) {
    bme_ok = true;
    Serial.println("✅ BME280 Connected at 0x76.");
  } else {
    Serial.println("⚠️ BME280 not found at 0x76. Trying 0x77...");
    if (bme.begin(0x77)) {
      bme_ok = true;
      Serial.println("✅ BME280 Connected at 0x77.");
    } else {
      Serial.println("❌ ERROR: BME280 Not Found. (Continuing without it)");
      bme_ok = false;
    }
  }

  // C. Attach Motors (explicit pulse range helps some ESCs)
  Serial.println("...Attaching Motors...");
  m1.attach(PIN_M1_FL, MIN_THROTTLE, MAX_THROTTLE);
  m2.attach(PIN_M2_FR, MIN_THROTTLE, MAX_THROTTLE);
  m3.attach(PIN_M3_BR, MIN_THROTTLE, MAX_THROTTLE);
  m4.attach(PIN_M4_BL, MIN_THROTTLE, MAX_THROTTLE);

  // D. Arming
  Serial.println("WARNING: ARMING MOTORS IN 5 SECONDS.");
  writeAllMotors(MIN_THROTTLE);
  delay(5000);

  // If IMU is missing, do NOT fly.
  if (!imu_ok) {
    Serial.println("🚫 IMU missing -> motors will stay STOPPED.");
  } else {
    Serial.println("✅ ARMED. Starting Flight Loop.");
  }

  last_us = micros();
}

// --- 5. LOOP ---
void loop() {
  // Run a consistent loop rate (~250 Hz => 4000 us)
  static unsigned long last_loop_us = 0;
  if (micros() - last_loop_us < 4000) return;
  last_loop_us = micros();

  // If no IMU, keep motors off safely
  if (!imu_ok) {
    writeAllMotors(MIN_THROTTLE);
    static unsigned long last_warn = 0;
    if (millis() - last_warn > 500) {
      Serial.println("🚫 Waiting for IMU... motors stopped.");
      last_warn = millis();
    }
    return;
  }

  // A. Read Sensors
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // dt in seconds
  unsigned long now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.004f; // clamp weird dt spikes

  // Accel angles (more stable formulas)
  const float ax = a.acceleration.x;
  const float ay = a.acceleration.y;
  const float az = a.acceleration.z;

  // Avoid divide-by-zero edge cases
  float accel_pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI;
  float accel_roll  = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

  // Gyro rates (rad/s -> deg/s)
  float gyro_pitch_rate = g.gyro.x * 180.0f / PI; // axis mapping may differ by board orientation
  float gyro_roll_rate  = g.gyro.y * 180.0f / PI;

  // Complementary filter
  pitch_deg = CF_ALPHA * (pitch_deg + gyro_pitch_rate * dt) + (1.0f - CF_ALPHA) * accel_pitch;
  roll_deg  = CF_ALPHA * (roll_deg  + gyro_roll_rate  * dt) + (1.0f - CF_ALPHA) * accel_roll;

  if (isBad(pitch_deg) || isBad(roll_deg)) {
    writeAllMotors(MIN_THROTTLE);
    Serial.println("🚨 EMERGENCY: Bad IMU data (NaN/Inf)");
    return;
  }

  // B. Safety Cutoff
  if (fabsf(pitch_deg) > MAX_TILT_ANGLE || fabsf(roll_deg) > MAX_TILT_ANGLE) {
    writeAllMotors(MIN_THROTTLE);
    Serial.println("🚨 EMERGENCY: TILT LIMIT EXCEEDED");
    return;
  }

  // C. Calculate Motor Mixing (P only)
  float pitch_adjust = pitch_deg * P_GAIN_PITCH;
  float roll_adjust  = roll_deg  * P_GAIN_ROLL;

  int speed_FL = (int)(BASE_THROTTLE + pitch_adjust + roll_adjust);
  int speed_FR = (int)(BASE_THROTTLE + pitch_adjust - roll_adjust);
  int speed_BR = (int)(BASE_THROTTLE - pitch_adjust - roll_adjust);
  int speed_BL = (int)(BASE_THROTTLE - pitch_adjust + roll_adjust);

  // D. Limits
  speed_FL = constrain(speed_FL, MIN_THROTTLE, MAX_THROTTLE);
  speed_FR = constrain(speed_FR, MIN_THROTTLE, MAX_THROTTLE);
  speed_BR = constrain(speed_BR, MIN_THROTTLE, MAX_THROTTLE);
  speed_BL = constrain(speed_BL, MIN_THROTTLE, MAX_THROTTLE);

  // E. Send to Motors
  m1.writeMicroseconds(speed_FL);
  m2.writeMicroseconds(speed_FR);
  m3.writeMicroseconds(speed_BR);
  m4.writeMicroseconds(speed_BL);

  // F. Debug Print
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    Serial.print("Pitch: "); Serial.print(pitch_deg, 1);
    Serial.print(" Roll: "); Serial.print(roll_deg, 1);
    Serial.print(" | FL: "); Serial.print(speed_FL);
    Serial.print(" FR: "); Serial.print(speed_FR);
    Serial.print(" BR: "); Serial.print(speed_BR);
    Serial.print(" BL: "); Serial.println(speed_BL);
    last_print = millis();
  }
}
