#ifndef HighGAccel_H
#define HighGAccel_H

#include <Arduino.h>
#include <SPI.h>

class HighGAccel {
  public:
    HighGAccel(int csPin); // Constructor
    bool begin();   // Bring-up and configure registers
    void update();  // Read the latest data

    // Public read/write tools
    void writeRegister(uint8_t regAddress, uint8_t data);
    uint8_t readRegister(uint8_t regAddress);
    
    // ============================================================
    // RAW CHIP-FRAME READINGS
    // KX134 chip's native coordinate system. Diagnostics only.
    // ============================================================
    float accelX, accelY, accelZ;   // g's, chip frame

    // ============================================================
    // ROCKET-FRAME ACCESSORS
    // ============================================================
    // ⚠ NOT VERIFIED ON BENCH YET — KX134 chip orientation may
    //   differ from IMU due to PCB layout. Test nose-up before
    //   trusting these values.
    //
    // TODO: Tentative mapping (matches IMU until proven otherwise):
    //   Rocket Axial = Chip +Y
    //   Rocket Lat-1 = Chip +X
    //   Rocket Lat-2 = Chip +Z
    // ============================================================
    inline float axialAccelG() const { return accelY; }
    inline float lat1AccelG()  const { return accelX; }
    inline float lat2AccelG()  const { return accelZ; }

  private:
    int _csPin;
};

#endif