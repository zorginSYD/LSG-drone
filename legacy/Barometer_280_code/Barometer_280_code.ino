#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
/* å */
/*
 * WIRING:
 * VCC -> 3.3V
 * GND -> GND
 * SDA -> A4
 * SCL -> A5
 */

Adafruit_BME280 bme; // Create BME280 object

void setup() {
  Serial.begin(9600);
  Serial.println("BME280 Sensor Test");

  // We explicitly start it at address 0x76
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor!");
    Serial.println("If this fails, your sensor might be a BMP280 clone with a weird ID.");
    while (1);
  }
  
  Serial.println("Sensor found!");
}

void loop() {
  Serial.print("Altitude: ");
  Serial.print(bme.readAltitude(1013.25)); // 1013.25 is standard sea level pressure 
  // change the above based on the location
  Serial.println(" m");

  delay(200);
}