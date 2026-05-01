#include <Arduino.h>
#include <Servo.h>

Servo testServo; 

// ---> CHANGE THIS TO THE ACTUAL TEENSY PIN CONNECTED TO YOUR SERVO <---
const int SERVO_PIN = 29;

void setup() {
  pinMode(4, OUTPUT);  digitalWrite(4, HIGH);
  // Start standard USB Serial just in case you want to monitor it on your desk
  Serial.begin(115200);
  
  // Give the power rails 1 second to stabilize after battery plug-in
  delay(1000);
  while (!Serial && millis() < 10000) {};
  Serial.println("Standalone Servo Test Initialized.");

  // Attach the servo to the pin
  testServo.attach(SERVO_PIN);
  
  // Command it to a known starting position (0 degrees)
  testServo.write(0);
  delay(1000); 
}

void loop() {
  // Move to 90 degrees
  Serial.println("Sweeping to 90...");
  testServo.write(90);
  delay(1500); // Wait 1.5 seconds for it to mechanically reach the position
  digitalWrite(4, LOW); // Toggle pin 4 LOW to indicate servo reached 90 degrees

  // Move back to 0 degrees
  Serial.println("Sweeping to 0...");
  testServo.write(0);
  delay(1500); // Wait 1.5 seconds
  digitalWrite(4, HIGH); // Toggle pin 4 HIGH to indicate servo reached 0 degrees
}