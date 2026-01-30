#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

// --- ESP32 I2C PINS ---
#define I2C_SDA 35
#define I2C_SCL 32

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
const float CF_ALPHA = 0.98f; 

// --- STABILIZATION ---
const float P_GAIN = 2.0f; 
const float MAX_TILT = 45.0f; 

// --- BARO VARS ---
float ground_pressure = 1013.25f;

void setup() {
  Serial.begin(115200); // ESP32 is better at higher baud rates
  
  // 1. I2C SETUP (ESP32 specific pins)
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. MOTORS SETUP
  // Use requested pins: 5, 18, 19, 21
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  motor1.setPeriodHertz(50);    // Standard Servo frequency
  motor1.attach(5, 1000, 2000);
  motor2.attach(18, 1000, 2000);
  motor3.attach(19, 1000, 2000);
  motor4.attach(21, 1000, 2000);

  // Arming sequence
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  Serial.println("--- ESP32 MOTOR TEST & STABILIZATION ---");
  Serial.println("ESCs Arming... Wait 2 seconds.");
  delay(2000);
  Serial.println("Enter throttle (1000-2000) via Serial Monitor.");

  // 3. SENSOR SETUP
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

  if (!bme.begin(0x76)) {
     if (!bme.begin(0x77)) {
        Serial.println("❌ BME280 Not Found!");
     } else {
        baro_ok = true;
     }
  } else {
    baro_ok = true;
  }

  if (baro_ok) {
    delay(100);
    ground_pressure = bme.readPressure() / 100.0F;
    Serial.println("✅ BME280 Ready.");
  }
}

void loop() {
  // --- A. SERIAL INPUT ---
  if (Serial.available() > 0) {
    int input = Serial.parseInt(); 
    if (input >= 1000 && input <= 2000) {
      throttle = input;
      Serial.print(">>> BASE THROTTLE: "); Serial.println(throttle);
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

    float accel_pitch = atan2f(a.acceleration.y, sqrtf(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
    float accel_roll  = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;
    float gyro_pitch_rate = g.gyro.x * 180.0f / PI;
    float gyro_roll_rate  = g.gyro.y * 180.0f / PI;

    pitch_deg = CF_ALPHA * (pitch_deg + gyro_pitch_rate * dt) + (1.0f - CF_ALPHA) * accel_pitch;
    roll_deg  = CF_ALPHA * (roll_deg  + gyro_roll_rate  * dt) + (1.0f - CF_ALPHA) * accel_roll;
  }

  // --- C. STABILIZATION ---
  int m1_val, m2_val, m3_val, m4_val;

  if (throttle > 1050 && imu_ok) {
    if (abs(pitch_deg) > MAX_TILT || abs(roll_deg) > MAX_TILT) {
      throttle = 1000;
      Serial.println("🚨 EMERGENCY CUTOFF");
    }
    
    float pitch_corr = (0.0f - pitch_deg) * P_GAIN;
    float roll_corr = (0.0f - roll_deg) * P_GAIN;

    m1_val = throttle + (int)pitch_corr + (int)roll_corr;
    m2_val = throttle + (int)pitch_corr - (int)roll_corr;
    m3_val = throttle - (int)pitch_corr - (int)roll_corr;
    m4_val = throttle - (int)pitch_corr + (int)roll_corr;
  } else {
    m1_val = m2_val = m3_val = m4_val = throttle;
  }

  // Write
  motor1.writeMicroseconds(constrain(m1_val, 1000, 2000));
  motor2.writeMicroseconds(constrain(m2_val, 1000, 2000));
  motor3.writeMicroseconds(constrain(m3_val, 1000, 2000));
  motor4.writeMicroseconds(constrain(m4_val, 1000, 2000));

  // --- D. DEBUG ---
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) { 
    last_print = millis();
    Serial.print("P:"); Serial.print(pitch_deg, 1);
    Serial.print(" R:"); Serial.print(roll_deg, 1);
    Serial.print(" | M1:"); Serial.print(m1_val);
    Serial.print(" M2:"); Serial.print(m2_val);
    Serial.print(" M3:"); Serial.print(m3_val);
    Serial.print(" M4:"); Serial.println(m4_val);
  }
}