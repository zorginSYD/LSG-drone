#include <Servo.h>

// Define the 4 motors
Servo motor1, motor2, motor3, motor4;

int throttle = 1000; // Starting pulse width (1000us is usually 'off')

void setup() {
  Serial.begin(9600);
  
  // Attach motors to PWM pins (D3, D5, D6, D9 are common on Uno/Nano)
  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(7);
  motor4.attach(10);

  // Send "Stop" signal to arm the ESCs
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  Serial.println("ESCs Arming... Wait 2 seconds.");
  delay(2000);
  Serial.println("Enter a value between 1000 and 2000:");
}

void loop() {
  if (Serial.available() > 0) {
    int input = Serial.parseInt(); // Read the number from Serial Monitor

    // Check if the input is within a safe range
    if (input >= 1000 && input <= 2000) {
      throttle = input;
      Serial.print("Setting Throttle to: ");
      Serial.println(throttle);
      
      // Update all motors
      motor1.writeMicroseconds(throttle);
      motor2.writeMicroseconds(throttle);
      motor3.writeMicroseconds(throttle);
      motor4.writeMicroseconds(throttle);
    } else if (input != 0) {
      Serial.println("Invalid input! Use 1000 - 2000.");
    }
  }
}