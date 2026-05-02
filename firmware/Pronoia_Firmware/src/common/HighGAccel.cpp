#include "HighGAccel.h"

// KX134 Register Map (From your TRM)
#define KX134_WHO_AM_I  0x13
#define KX134_CNTL1     0x1B
#define KX134_CNTL2     0x1C
#define KX134_ODCNTL    0x21
#define KX134_XOUT_L    0x08 // Start of data registers

HighGAccel::HighGAccel(int csPin) {
  _csPin = csPin;
}

bool HighGAccel::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  writeRegister(KX134_CNTL1, 0x00); // Put into Standby Mode to configure
  delay(10); 

  writeRegister(KX134_CNTL2, 0b10000000); // Software Reset
  delay(10);

  if (readRegister(KX134_WHO_AM_I) != 0x46) {
    return false;
  }

  // 3. Set Output Data Rate (ODCNTL) to 100Hz (0x06 is 100Hz in the TRM)
  writeRegister(KX134_ODCNTL, 0b00001010); // ODR = 800 Hz, IIR Filter is not bypassed, IIR Filter Corner Frequency is ODR/9

  writeRegister(KX134_CNTL1, 0b11011000); // Standby -> Operating, High-Res, +-64g
  
  delay(1300); // Delay: 2 ms to 1300 ms before acceleration outputs are valid
  return true;
}

void HighGAccel::update() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  
  // Send the starting address with the READ bit (0x80) set
  SPI.transfer(KX134_XOUT_L | 0x80); 
  
  // KX134 supports auto-increment by default, clock in all 6 bytes
  uint8_t ax_l = SPI.transfer(0x00);
  uint8_t ax_h = SPI.transfer(0x00);
  uint8_t ay_l = SPI.transfer(0x00);
  uint8_t ay_h = SPI.transfer(0x00);
  uint8_t az_l = SPI.transfer(0x00);
  uint8_t az_h = SPI.transfer(0x00);

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // Reconstruct the 16-bit signed integers (KX134 is Little-Endian)
  int16_t rawX = (ax_h << 8) | ax_l;
  int16_t rawY = (ay_h << 8) | ay_l;
  int16_t rawZ = (az_h << 8) | az_l;

  // Convert to real G-forces (512 LSB/g for a 64g sensor)
  accelX = rawX / 512.0;
  accelY = rawY / 512.0;
  accelZ = rawZ / 512.0;
}

void HighGAccel::writeRegister(uint8_t regAddress, uint8_t data) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(regAddress); // Write does NOT have MSB set
  SPI.transfer(data);       
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

uint8_t HighGAccel::readRegister(uint8_t regAddress) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(regAddress | 0x80); // Read has MSB set
  uint8_t response = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  return response;
}