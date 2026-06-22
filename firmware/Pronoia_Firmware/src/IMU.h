#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>

class IMU {
  public:
    IMU(int csPin); // Constructor
    bool begin();   // Bring-up and configure registers
    void update();  // Read the latest data
    void calibrateGyro(int samples);
    
    // ============================================================
    // RAW CHIP-FRAME READINGS
    // These are in the IMU chip's native coordinate system.
    // Use these ONLY for diagnostics, calibration verification,
    // or low-level debugging. DO NOT use these in flight code.
    // ============================================================
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;

    // ============================================================
    // ROCKET-FRAME ACCESSORS — USE THESE EVERYWHERE IN FLIGHT CODE
    // ============================================================
    // Axis mapping for Prometheus avionics PCB:
    //   Rocket Axial  (along rocket length, +up)                     = Chip +Y
    //   Rocket Lat-1  (to the right of the rocket, when facing it)   = Chip +X
    //   Rocket Lat-2  (pointed towards you, when facing the rocket)  = Chip +Z
    //
    // ⚠ If PCB layout changes, update ONLY this section.
    //   Verify on bench: nose-up should give axialAccelG() ≈ +1.0
    // ============================================================
    inline float axialAccelG() const   { return  accelY; }
    inline float lat1AccelG()  const   { return  accelX; }
    inline float lat2AccelG()  const   { return  accelZ; }

    // Gyro rotation axis convention:
    //   Roll  = rotation about rocket axial axis = gyroY // A positive roll is counterclockwise rotation when looking at the rocket axially from the base
    //   Pitch = rotation about Lat-1 axis        = gyroX // A positive pitch is counterclockwise rotation when looking at the rocket from the right side
    //   Yaw   = rotation about Lat-2 axis        = gyroZ // A positive yaw is a counterclockwise rotation when looking at the rocket from "the front"
    inline float rollRateDps()  const  { return  gyroY; }
    inline float pitchRateDps() const  { return  gyroX; }
    inline float yawRateDps()   const  { return  gyroZ; }

    // Accessors for the gyro bias offsets (after calibration).
    // Useful for the CAL_ALL CLI to print the measured biases so the operator
    // can sanity-check them and the log can record them.
    inline float gyroOffsetX() const { return _gyroX_off; }
    inline float gyroOffsetY() const { return _gyroY_off; }
    inline float gyroOffsetZ() const { return _gyroZ_off; }

  private:
    int _csPin;

    // Offset storage
    float _gyroX_off = 0;
    float _gyroY_off = 0;
    float _gyroZ_off = 0;

    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
};

#endif