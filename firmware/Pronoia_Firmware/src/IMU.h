#ifndef TOKMAS_IMU_H
#define TOKMAS_IMU_H

#include <Arduino.h>
#include <SPI.h>

class IMU {
  public:
    IMU(int csPin); // Constructor
    bool begin();   // Bring-up and configure registers
    void update();  // Read the latest data
    
    // Public variables to hold your data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;

  private:
    int _csPin;
    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
};

#endif