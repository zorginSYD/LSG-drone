#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

// --- MOTORS ---
Servo motor1, motor2, motor3, motor4;
int throttle = 1000; 

// --- SENSORS ---
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
bool imu_ok = false;
bool baro_ok = false;

// --- IMU VARS ---
float pitch_deg = 0.0f;
float roll_deg = 0.0f;
unsigned long last_imu_us = 0;
const float CF_ALPHA = 0.98f; // Complementary filter weight

// --- STABILIZATION ---
const float P_GAIN = 2.0f; // Proportional Gain: Higher = stronger reaction
// Safety: Stop motors if tilted too much
const float MAX_TILT = 45.0f; 

// --- BARO VARS ---
float ground_pressure = 1013.25f;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // 1. MOTORS SETUP
  // Pins: 3, 5, 7, 12
  // ASSUMPTION: 1=FL, 2=FR, 3=BR, 4=BL (Quad X)
  motor1.attach(3);
  motor2.attach(7);
  motor3.attach(10);
  motor4.attach(11);

  // Arming sequence
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  Serial.println("--- MOTOR TEST & STABILIZATION ---");
  Serial.println("ESCs Arming... Wait 2 seconds.");
  delay(2000);
  Serial.println("Enter throttle (1000-2000) to spin motors.");
  Serial.println("Stabilization is ACTIVE. Tilt the drone to see motor speed changes.");

  // 2. SENSOR SETUP
  // Init MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 Not Found!");
  } else {
    imu_ok = true;
    Serial.println("✅ MPU6050 Ready.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  last_imu_us = micros();

  // Init BME280
  if (!bme.begin(0x76)) {
     if (!bme.begin(0x77)) {
        Serial.println("❌ BME280 Not Found!");
     }
  } else {
    baro_ok = true;
    Serial.println("✅ BME280 Ready (0x76).");
  }

  if (baro_ok) {
    delay(100);
    ground_pressure = bme.readPressure() / 100.0F; // Set 0m reference
    Serial.print("Ground Pressure: "); Serial.print(ground_pressure); Serial.println(" hPa");
  }
}

void loop() {
  // --- A. SERIAL INPUT ---
  if (Serial.available() > 0) {
    int input = Serial.parseInt(); 
    if (input >= 1000 && input <= 2000) {
      throttle = input;
      Serial.print(">>> BASE THROTTLE: "); Serial.println(throttle);
    } else if (input != 0) {
      Serial.println("Invalid input! Use 1000 - 2000.");
    }
  }

  // --- B. SENSOR UPDATES ---
  if (imu_ok) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now_us = micros();
    float dt = (now_us - last_imu_us) * 1e-6f;
    last_imu_us = now_us;
    
    if (dt <= 0.0f || dt > 0.1f) dt = 0.004f;

    // Calculate Accel Angles
    float accel_pitch = atan2f(a.acceleration.y, sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
    float accel_roll  = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

    // Calculate Gyro Rates
    float gyro_pitch_rate = g.gyro.x * 180.0f / PI;
    float gyro_roll_rate  = g.gyro.y * 180.0f / PI;

    // Complementary Filter
    pitch_deg = CF_ALPHA * (pitch_deg + gyro_pitch_rate * dt) + (1.0f - CF_ALPHA) * accel_pitch;
    roll_deg  = CF_ALPHA * (roll_deg  + gyro_roll_rate  * dt) + (1.0f - CF_ALPHA) * accel_roll;
  }

  // --- C. STABILIZATION CALCULATION ---
  int m1_val = 1000;
  int m2_val = 1000;
  int m3_val = 1000;
  int m4_val = 1000;

  // Only stabilize if throttle is raised (safety)
  if (throttle > 1050 && imu_ok) {
    // Safety Cutoff
    if (abs(pitch_deg) > MAX_TILT || abs(roll_deg) > MAX_TILT) {
      throttle = 1000; // Kill motors
      Serial.println("🚨 EMERGENCY: MAX TILT EXCEEDED!");
    } else {
      // P-Controller
      // Error = Target (0) - Current
      float pitch_error = 0.0f - pitch_deg; 
      float roll_error = 0.0f - roll_deg;

      float pitch_correction = pitch_error * P_GAIN;
      float roll_correction = roll_error * P_GAIN;

      // Mixing (Quad X)
      // M1 (FL) = Thr + Pitch + Roll
      // M2 (FR) = Thr + Pitch - Roll
      // M3 (BR) = Thr - Pitch - Roll
      // M4 (BL) = Thr - Pitch + Roll
      
      m1_val = throttle + (int)pitch_correction + (int)roll_correction;
      m2_val = throttle + (int)pitch_correction - (int)roll_correction;
      m3_val = throttle - (int)pitch_correction - (int)roll_correction;
      m4_val = throttle - (int)pitch_correction + (int)roll_correction;
    }
  } else {
      // If throttle low, just idle/off, no stabilization
      m1_val = throttle;
      m2_val = throttle;
      m3_val = throttle;
      m4_val = throttle;
  }

  // Constrain
  m1_val = constrain(m1_val, 1000, 2000);
  m2_val = constrain(m2_val, 1000, 2000);
  m3_val = constrain(m3_val, 1000, 2000);
  m4_val = constrain(m4_val, 1000, 2000);

  // Write to motors
  motor1.writeMicroseconds(m1_val);
  motor2.writeMicroseconds(m2_val);
  motor3.writeMicroseconds(m3_val);
  motor4.writeMicroseconds(m4_val);

  // --- D. DISPLAY DATA ---
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) { 
    last_print = millis();
    
    Serial.print("P: "); Serial.print(pitch_deg, 1);
    Serial.print(" R: "); Serial.print(roll_deg, 1);
    Serial.print(" | M1: "); Serial.print(m1_val);
    Serial.print(" M2: "); Serial.print(m2_val);
    Serial.print(" M3: "); Serial.print(m3_val);
    Serial.print(" M4: "); Serial.print(m4_val);
    
    if (baro_ok) {
      float alt = bme.readAltitude(ground_pressure);
      Serial.print(" | Alt: "); Serial.print(alt, 1);
    }
    Serial.println();
  }
}