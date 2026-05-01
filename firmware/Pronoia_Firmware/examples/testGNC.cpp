#include <Arduino.h>
#include <SPI.h>
#include "IMU.h"
#include "HighGAccel.h"
#include "Barometer.h"

IMU imu(10);
HighGAccel highG(37); 
Barometer baro(0); // Pin 0 for Barometer CS

void setup() {
  delay(500); // Wait a moment for the system to stabilize after power-up
  // Pull ALL Chip Selects HIGH immediately
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(37, OUTPUT); digitalWrite(37, HIGH);
  pinMode(0, OUTPUT);  digitalWrite(0, HIGH);
  pinMode(4, OUTPUT);  digitalWrite(4, HIGH);

  Serial.begin(115200);
  SPI.begin();   // Start SPI0 for IMU and High-G Accel
  SPI1.begin();  // Start SPI1 for Barometer!
  
  while (!Serial && millis() < 10000);

  Serial.println("\n=========================================");
  Serial.println("   PRONOIA PCBA HARDWARE TEST SCRIPT   ");
  Serial.println("=========================================\n");

  Serial.println("\n--- Initializing Sensors ---");

  // --- 1. IMU INITIALIZATION ---
  Serial.print("Checking IMU (BMI270)... ");
  if (imu.begin()) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL! Check SPI0 wiring.");
  }

  // --- 2. HIGH-G INITIALIZATION ---
  Serial.print("Checking High-G (KX134)... ");
  if (highG.begin()) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL! Check SPI0 wiring.");
  }

  // --- 3. BAROMETER INITIALIZATION & CALIBRATION ---
  Serial.print("Checking Barometer (MS5611)... ");
  if (baro.begin()) {
    Serial.println("PASS");
    Serial.println("Calibrating Barometer (100 samples)...");
    baro.calibrateBaro(100);
    Serial.print("Launchpad MSL: ");
    Serial.print(baro.groundAltitude_MSL);
    Serial.println(" m");

    Serial.println("\n--- MS5611 Factory Coefficients ---");
    Serial.print("C1 (Sens): "); Serial.println(baro.C1);
    Serial.print("C2 (Off):  "); Serial.println(baro.C2);
    Serial.print("C3 (TCS):  "); Serial.println(baro.C3);
    Serial.print("C4 (TCO):  "); Serial.println(baro.C4);
    Serial.print("C5 (Tref): "); Serial.println(baro.C5);
    Serial.print("C6 (Temp): "); Serial.println(baro.C6);
    Serial.println("-----------------------------------\n");

  } else {
    Serial.println("FAIL! Check SPI1 wiring.");
  }
  
  Serial.println("\n>>> ALL SENSORS READY <<<");
  Serial.println("Type 'c' and hit ENTER in the serial monitor to calibrate Gyro.\n");
  delay(1000);
}

void loop() {
  // 1. Grab the freshest data non-blocking style
  imu.update();
  highG.update();
  baro.update(); 

  // 2. Check for Serial Commands (Gyro Calibration)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c') {
      Serial.println("\n>>> CALIBRATING GYRO... DO NOT MOVE BOARD <<<");
      imu.calibrateGyro(200); 
      Serial.println(">>> CALIBRATION COMPLETE <<<\n");
    }
  }

  // 3. Print the data at 10 Hz (every 100ms) using printf for perfect alignment
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    digitalWrite(4, !digitalRead(4));
    
    Serial.printf("IMU[X:%5.2f Y:%5.2f Z:%5.2f g] | GYR[X:%6.1f Y:%6.1f Z:%6.1f dps] | HiG[X:%6.2f Y:%6.2f Z:%6.2f g] | BARO[AGL:%6.1f m Temp:%5.1f C Press:%6.1f mbar]\n",
                  imu.accelX, imu.accelY, imu.accelZ,
                  imu.gyroX, imu.gyroY, imu.gyroZ,
                  highG.accelX, highG.accelY, highG.accelZ,
                  baro.altitude_AGL, baro.temp_C, baro.pressure_Mbar);

    lastPrint = millis();
  }
}