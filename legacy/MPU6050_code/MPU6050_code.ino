/*
 * ======================================================================================
 * PROJECT WIRING CONFIGURATION
 * Board: Arduino Uno
 * Protocol: I2C (Inter-Integrated Circuit)
 * ======================================================================================
 * * MASTER I2C BUS CONNECTIONS (Arduino Uno):
 * -----------------------------------------
 * SDA (Data)  : Pin A4  -> Connected to SDA on both sensors
 * SCL (Clock) : Pin A5  -> Connected to SCL on both sensors
 * GND         : GND     -> Common Ground Rail
 * * SENSOR 1: GY-521 (MPU-6050 Accelerometer/Gyro)
 * ----------------------------------------------
 * VCC         : 5V (The GY-521 usually has a regulator)
 * GND         : GND
 * SDA         : Connected to Arduino A4
 * SCL         : Connected to Arduino A5
 * I2C Address : 0x68 (Default)
 * * SENSOR 2: BME280 (Temperature/Pressure/Altitude)
 * ------------------------------------------------
 * VCC/VIN     : 3.3V (CRITICAL: Connect to 3.3V pin to protect sensor)
 * GND         : GND
 * SDA         : Connected to Arduino A4
 * SCL         : Connected to Arduino A5
 * I2C Address : 0x76 (Common) or 0x77
 * * NOTE: 
 * Since both sensors share A4 and A5, use a breadboard rail to 
 * create a junction for the SDA wires and another for the SCL wires.
 * ======================================================================================
 */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200); // Note: Set Serial Monitor to 115200 baud!
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");

  // Setup basic performance settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Calculate Pitch & Roll from Accelerometer Data
     (Standard Aerospace formulas)
  */
  float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float roll  = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;

  /* ----- VISUALIZATION ----- 
     We print a "Bubble Level" style graphic
  */
  
  Serial.print("PITCH (Front/Back): ");
  Serial.print(pitch, 1); // Print angle with 1 decimal
  Serial.print(" deg  |  ");

  Serial.print("ROLL (Left/Right): ");
  Serial.print(roll, 1);
  Serial.print(" deg  |  ");
  
  Serial.print("STATUS: ");
  
  // LOGIC TO DETERMINE DIRECTION
  // We use a "threshold" of 3 degrees. Inside 3 degrees = LEVEL.
  int threshold = 3;
  
  bool isLevel = true; // Assume level first
  
  // CHECK FRONT / BACK
  if (pitch < -threshold) {
    Serial.print("Tilted FRONT ");
    isLevel = false;
  } else if (pitch > threshold) {
    Serial.print("Tilted BACK ");
    isLevel = false;
  }
  
  // CHECK LEFT / RIGHT
  if (roll > threshold) {
    Serial.print("RIGHT");
    isLevel = false;
  } else if (roll < -threshold) {
    Serial.print("LEFT");
    isLevel = false;
  }

  // IF FLAT
  if (isLevel) {
    Serial.print(">> PERFECTLY LEVEL (ZERO) <<");
  }

  Serial.println(""); // New line
  
  delay(100); // Speed of update
}