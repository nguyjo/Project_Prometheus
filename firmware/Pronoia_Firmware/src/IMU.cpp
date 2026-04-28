#include "IMU.h"

IMU::IMU(int csPin) {
  _csPin = csPin;
}

bool IMU::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  // Configure the IMU's registers here using writeRegister()
  // See the Tokmas (Chinese Clone) ICM-42688-PC datasheet for the CTRL Register Bit Definitions
  writeRegister(0x02, 0b01100000);
  writeRegister(0x03, 0b00110011);
  writeRegister(0x04, 0b01110011);
  writeRegister(0x06, 0b01010101);

  
  // Verify WHO_AM_I
  if (readRegister(0x00) == 0x05) return true;
  return false;
}

void IMU::update() {
  // Read the 6 bytes of accel data and 6 bytes of gyro data
  // Combine the High and Low bytes, convert to G's and Deg/s, 
  // and store them in accelX, gyroX, etc.

}

// Helper function to WRITE a byte to a register
void IMU::writeRegister(uint8_t regAddress, uint8_t data) {
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  
  // Send the register address (MSB is 0 for Write)
  SPI.transfer(regAddress); 
  
  // Send the configuration byte
  SPI.transfer(data);       
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

// Helper function to READ a byte from a register (to verify)
uint8_t IMU::readRegister(uint8_t regAddress) {
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  
  // Send the register address with the Read Bit (0x80) set
  SPI.transfer(regAddress | 0x80); 
  uint8_t response = SPI.transfer(0x00);
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  
  return response;
}