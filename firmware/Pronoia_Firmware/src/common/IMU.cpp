#include "IMU.h"

IMU::IMU(int csPin) {
  _csPin = csPin;
}

bool IMU::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  // 1. Soft Reset to clear out any bad state
  writeRegister(0x60, 0xB0);
  delay(20); // Give the IMU time to reboot

  // Verify WHO_AM_I
  if (readRegister(0x00) != 0x05) {
    return false;
  }

  // Configure the IMU's registers here using writeRegister()
  // See the Tokmas (Chinese Clone) ICM-42688-PC datasheet for the CTRL Register Bit Definitions
  writeRegister(0x02, 0b01100000); // CTRL1: 4-wire SPI, Auto-inc ON, Big-Endian, disable INT pins
  writeRegister(0x03, 0b00110011); // CTRL2: Disable Accel self-test, Accel at +-16g, 896.8 Hz ODR
  writeRegister(0x04, 0b01110011); // CTRL3: Disable Gyro self-test, Gyro at +-2048dps, 896.8 Hz ODR
  writeRegister(0x06, 0b01010101); // CTRL5: Gyro and Accel Hardware LPF enabled at set to 48.33752 Hz
  writeRegister(0x08, 0b00000011); // CTRL7: Enable Accel and Gyro

  delay(50);
  return true;
}

void IMU::calibrateGyro(int samples) {
  // Save and zero the current offset so we read raw bias
  const float savedX = _gyroX_off;
  const float savedY = _gyroY_off;
  const float savedZ = _gyroZ_off;
  _gyroX_off = _gyroY_off = _gyroZ_off = 0.0f;

  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    update();              // Now reads truly raw (no offset subtraction)
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(5);
  }

  _gyroX_off = sumX / (float)samples;
  _gyroY_off = sumY / (float)samples;
  _gyroZ_off = sumZ / (float)samples;
}

void IMU::update() {
  // Read the 6 bytes of accel data and 6 bytes of gyro data
  // Combine the High and Low bytes, convert to G's and Deg/s, 
  // and store them in accelX, gyroX, etc.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Can speed up to 1MHz now!
  digitalWrite(_csPin, LOW);
  
  // 1. Send the starting address (0x35 for AX_L) with the READ bit (0x80) set
  SPI.transfer(0x35 | 0x80); 
  
  // 2. Clock in all 12 bytes sequentially 
  uint8_t ax_l = SPI.transfer(0x00); // 0x35
  uint8_t ax_h = SPI.transfer(0x00); // 0x36
  uint8_t ay_l = SPI.transfer(0x00); // 0x37
  uint8_t ay_h = SPI.transfer(0x00); // 0x38
  uint8_t az_l = SPI.transfer(0x00); // 0x39
  uint8_t az_h = SPI.transfer(0x00); // 0x3A
  
  uint8_t gx_l = SPI.transfer(0x00); // 0x3B
  uint8_t gx_h = SPI.transfer(0x00); // 0x3C
  uint8_t gy_l = SPI.transfer(0x00); // 0x3D
  uint8_t gy_h = SPI.transfer(0x00); // 0x3E
  uint8_t gz_l = SPI.transfer(0x00); // 0x3F
  uint8_t gz_h = SPI.transfer(0x00); // 0x40

  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // 3. Reconstruct the 16-bit integers using bitwise math (Shift High byte left by 8, OR with Low byte)
  // These measurements include a low-pass filter, set in IMU::begin(), at 48.33752 Hz
  int16_t rawAccelX = (ax_h << 8) | ax_l;
  int16_t rawAccelY = (ay_h << 8) | ay_l;
  int16_t rawAccelZ = (az_h << 8) | az_l;
  
  int16_t rawGyroX = (gx_h << 8) | gx_l;
  int16_t rawGyroY = (gy_h << 8) | gy_l;
  int16_t rawGyroZ = (gz_h << 8) | gz_l;
  
  // Convert to physical units and store in public class variables
  accelX = rawAccelX / 2048.0; // Results in Gs
  accelY = rawAccelY / 2048.0;
  accelZ = rawAccelZ / 2048.0;

  gyroX = (rawGyroX / 16.0) - _gyroX_off;     // Results in Degrees per Second
  gyroY = (rawGyroY / 16.0) - _gyroY_off;
  gyroZ = (rawGyroZ / 16.0) - _gyroZ_off;
}

// Helper function to WRITE a byte to a register
void IMU::writeRegister(uint8_t regAddress, uint8_t data) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
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
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  
  // Send the register address with the Read Bit (0x80) set
  SPI.transfer(regAddress | 0x80); 
  uint8_t response = SPI.transfer(0x00);
  
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  
  return response;
}