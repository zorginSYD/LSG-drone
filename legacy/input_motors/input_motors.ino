// --- REMOTEXY CONFIGURATION ---
#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>

// WiFi Credentials
#define REMOTEXY_WIFI_SSID "DRONE_GEMINI"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY Config (Joystick + Switch + Slider)
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = { 
  255,4,0,0,0,37,0,8,13,0, 
  5,49,60,10,30,30,1,26,31,2, // Joystick
  0,25,7,22,11,2,26,31,31,79, 78,0,79,70,70,0, // Switch
  4,0,9,14,7,41,2,26 // Slider
};

struct {
    // Inputs
    int8_t joystick_1_x; // -100..100
    int8_t joystick_1_y; // -100..100
    uint8_t switch_1;    // 1=ON, 0=OFF (Arming)
    int8_t slider_1;     // 0..100 (Throttle)

    // Internal
    uint8_t connect_flag; 
} RemoteXY;
#pragma pack(pop)
// ------------------------------

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <math.h>

// --- ESP32 I2C PINS ---
#define I2C_SDA 33
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

// --- FLIGHT TARGETS ---
float targetPitch = 0.0f;
float targetRoll = 0.0f;

// --- BARO VARS ---
float ground_pressure = 1013.25f;

void setup() {
  Serial.begin(115200); 
  delay(1000); 
  Serial.println("\n\n=== ESP32 DRONE BOOTING ===");

  // 1. REMOTEXY SETUP
  RemoteXY_Init();
  Serial.println("✅ RemoteXY Initialized (WiFi Point: DRONE_GEMINI)");

  // 2. I2C SETUP
  Wire.begin(I2C_SDA, I2C_SCL);

  // 3. MOTORS SETUP
  motor1.setPeriodHertz(50);    
  motor2.setPeriodHertz(50);  
  motor3.setPeriodHertz(50);  
  motor4.setPeriodHertz(50);  

  motor1.attach(5, 1000, 2000);
  motor2.attach(18, 1000, 2000);
  motor3.attach(19, 1000, 2000);
  motor4.attach(21, 1000, 2000);

  // Arming sequence
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(2000);

  // 4. SENSOR SETUP
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
  // --- A. REMOTEXY HANDLER ---
  RemoteXY_Handler();

  // --- B. MAP INPUTS ---
  if (RemoteXY.switch_1 == 1) { // ARMED
    // Map Slider (0-100) to Throttle (1000-2000)
    // Add a safe deadzone at bottom
    if (RemoteXY.slider_1 < 5) throttle = 1000;
    else throttle = map(RemoteXY.slider_1, 5, 100, 1050, 1800); // Cap at 1800 for safety

    // Map Joystick (-100..100) to Angles (-30..30 degrees)
    targetRoll = map(RemoteXY.joystick_1_x, -100, 100, -30, 30);
    targetPitch = map(RemoteXY.joystick_1_y, -100, 100, -30, 30);
    // Note: Joystick Y might need inversion depending on orientation

  } else { // DISARMED
    throttle = 1000;
    targetPitch = 0;
    targetRoll = 0;
  }

  // --- C. SENSOR UPDATES ---
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

  // --- D. STABILIZATION ---
  int m1_val, m2_val, m3_val, m4_val;

  if (throttle > 1050 && imu_ok) {
    // Safety Cutoff
    if (abs(pitch_deg) > MAX_TILT || abs(roll_deg) > MAX_TILT) {
      throttle = 1000;
      Serial.println("🚨 EMERGENCY: MAX TILT EXCEEDED!");
    }
    
    // P-Controller
    // Error = Target - Current
    float pitch_error = targetPitch - pitch_deg; 
    float roll_error = targetRoll - roll_deg;

    float pitch_corr = pitch_error * P_GAIN;
    float roll_corr = roll_error * P_GAIN;

    // Mixing (Quad X)
    // 1(FL), 2(FR), 3(BR), 4(BL)
    // Signs depend on exact motor/prop direction. Standard Quad X:
    // Pitch forward (+): Back motors speed up
    // Roll right (+): Left motors speed up
    
    // Adjust signs if drone flips!
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

  // --- E. DEBUG ---
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) { 
    last_print = millis();
    Serial.print("SW:"); Serial.print(RemoteXY.switch_1);
    Serial.print(" Thr:"); Serial.print(throttle);
    Serial.print(" | P:"); Serial.print(pitch_deg, 1);
    Serial.print(" R:"); Serial.print(roll_deg, 1);
    Serial.print(" | TgtP:"); Serial.print(targetPitch, 1);
    Serial.println();
  }
}
