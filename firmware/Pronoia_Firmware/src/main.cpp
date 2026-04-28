#include "IMU.h"

IMU imu(10); // CS pin is 10

void setup() {
  Serial.begin(115200);
  SPI.begin();
  
  if (!imu.begin()) {
    Serial.println("IMU Init Failed!");
    while(1); // Halt
  }
}

void loop() {
  imu.update(); // Grabs fresh data
  Serial.println(imu.accelZ); // Use the data!
}